#include "devices/timer.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include "threads/interrupt.h"
#include "threads/io.h"
#include "threads/synch.h"
#include "threads/thread.h"

/* timer.c 함수 목록 및 기능 설명
 *
 * // 초기화 및 보정 함수
 * void     timer_init(void);
 *     └ 8254 Programmable Interval Timer(PIT)를 TIMER_FREQ 주기로 설정하고
 *        타이머 인터럽트를 등록합니다.
 *
 * void     timer_calibrate(void);
 *     └ busy-wait 루프의 반복 횟수(loops_per_tick)를 측정하여
 *        짧은 지연(delay)을 정확히 구현할 수 있도록 보정합니다.
 *
 * // 시간 확인 함수
 * int64_t  timer_ticks(void);
 *     └ OS 부팅 이후 경과한 타이머 틱(tick) 수를 반환합니다.
 *
 * int64_t  timer_elapsed(int64_t then);
 *     └ 인자로 주어진 이전 틱 값부터 현재까지 경과한 틱 수를 계산하여 반환합니다.
 *
 * // 대기(슬립) 함수
 * void     timer_sleep(int64_t ticks);
 *     └ 지정한 틱 수만큼 현재 스레드를 블록(sleep) 상태로 대기시킵니다.
 *
 * void     timer_msleep(int64_t ms);
 *     └ 지정한 밀리초(ms)만큼 대기합니다. 내부적으로 real_time_sleep 사용.
 *
 * void     timer_usleep(int64_t us);
 *     └ 지정한 마이크로초(us)만큼 대기합니다. 내부적으로 real_time_sleep 사용.
 *
 * void     timer_nsleep(int64_t ns);
 *     └ 지정한 나노초(ns)만큼 대기합니다. 내부적으로 real_time_sleep 사용.
 *
 * void     timer_print_stats(void);
 *     └ 현재까지 누적된 전체 틱 수를 출력합니다.
 *
 * // 내부(static) 구현 함수
 * static void  timer_interrupt(struct intr_frame *args);
 *     └ 타이머 인터럽트 핸들러. 전역 ticks를 증가시키고
 *        thread_tick()을 호출하여 스케줄러에 알립니다.
 *
 * static bool  too_many_loops(unsigned loops);
 *     └ 주어진 루프 횟수로 busy-wait 했을 때 한 틱 이상 소요되는지 검사합니다.
 *
 * static void  busy_wait(int64_t loops);
 *     └ 인자로 받은 횟수만큼 단순 루프를 실행하여 매우 짧은 지연을 구현합니다.
 *
 * static void  real_time_sleep(int64_t num, int32_t denom);
 *     └ num/denom 초만큼 대기합니다. 틱 단위 이상이면 timer_sleep,
 *        이하 서브틱(sub-tick)은 busy_wait으로 처리합니다.
 */

/* See [8254] for hardware details of the 8254 timer chip. */

#if TIMER_FREQ < 19
#error 8254 timer requires TIMER_FREQ >= 19
#endif
#if TIMER_FREQ > 1000
#error TIMER_FREQ <= 1000 recommended
#endif

/* Number of timer ticks since OS booted. */
static int64_t ticks;

/* Number of loops per timer tick.
   Initialized by timer_calibrate(). */
static unsigned loops_per_tick;

static struct list sleeping_list;

static intr_handler_func timer_interrupt;
static bool too_many_loops (unsigned loops);
static void busy_wait (int64_t loops);
static void real_time_sleep (int64_t num, int32_t denom);

/* Sets up the 8254 Programmable Interval Timer (PIT) to
   interrupt PIT_FREQ times per second, and registers the
   corresponding interrupt. */
void
timer_init (void) {
	/* 8254 input frequency divided by TIMER_FREQ, rounded to
	   nearest. */
	uint16_t count = (1193180 + TIMER_FREQ / 2) / TIMER_FREQ;

	outb (0x43, 0x34);    /* CW: counter 0, LSB then MSB, mode 2, binary. */
	outb (0x40, count & 0xff);
	outb (0x40, count >> 8);
	list_init(&sleeping_list);

	intr_register_ext (0x20, timer_interrupt, "8254 Timer");
}

/* Calibrates loops_per_tick, used to implement brief delays. */
void
timer_calibrate (void) {
	unsigned high_bit, test_bit;

	ASSERT (intr_get_level () == INTR_ON);
	printf ("Calibrating timer...  ");

	/* Approximate loops_per_tick as the largest power-of-two
	   still less than one timer tick. */
	loops_per_tick = 1u << 10;
	while (!too_many_loops (loops_per_tick << 1)) {
		loops_per_tick <<= 1;
		ASSERT (loops_per_tick != 0);
	}

	/* Refine the next 8 bits of loops_per_tick. */
	high_bit = loops_per_tick;
	for (test_bit = high_bit >> 1; test_bit != high_bit >> 10; test_bit >>= 1)
		if (!too_many_loops (high_bit | test_bit))
			loops_per_tick |= test_bit;

	printf ("%'"PRIu64" loops/s.\n", (uint64_t) loops_per_tick * TIMER_FREQ);
}

/* Returns the number of timer ticks since the OS booted. */
int64_t
timer_ticks (void) {
	enum intr_level old_level = intr_disable ();
	int64_t t = ticks;
	intr_set_level (old_level);
	barrier ();
	return t;
}

/* Returns the number of timer ticks elapsed since THEN, which
   should be a value once returned by timer_ticks(). */
int64_t
timer_elapsed (int64_t then) {
	return timer_ticks () - then;
}

/* Suspends execution for approximately TICKS timer ticks. */
static bool wake_up_cmp(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED)
{
	struct thread *a1 = list_entry(a, struct thread, elem);
	struct thread *b1 = list_entry(b, struct thread, elem);
	return a1->wake_tick < b1->wake_tick;
}

void
timer_sleep (int64_t ticks) {
	int64_t start = timer_ticks ();
	if (ticks <= 0)return;

	enum intr_level old_level = intr_disable();

	struct thread *t = thread_current();
	t->wake_tick = start + ticks;

	list_insert_ordered(&sleeping_list, &t->elem, wake_up_cmp, NULL);
	thread_block();
	intr_set_level(old_level);
}



/* Suspends execution for approximately MS milliseconds. */
void
timer_msleep (int64_t ms) {
	real_time_sleep (ms, 1000);
}

/* Suspends execution for approximately US microseconds. */
void
timer_usleep (int64_t us) {
	real_time_sleep (us, 1000 * 1000);
}

/* Suspends execution for approximately NS nanoseconds. */
void
timer_nsleep (int64_t ns) {
	real_time_sleep (ns, 1000 * 1000 * 1000);
}

/* Prints timer statistics. */
void
timer_print_stats (void) {
	printf ("Timer: %"PRId64" ticks\n", timer_ticks ());
}

/* Timer interrupt handler. */
static void
timer_interrupt(struct intr_frame *args UNUSED)
{
	ticks++;
	thread_tick();

	struct list_elem *e = list_begin(&sleeping_list);
	while (e != list_end(&sleeping_list))
	{
		struct thread *t = list_entry(e, struct thread, elem);

		if (t->wake_tick <= timer_ticks())
		{
			e = list_remove(e);
			thread_unblock(t);
		}
		else
		{
			break;
		}
	}
}

/* Returns true if LOOPS iterations waits for more than one timer
   tick, otherwise false. */
static bool
too_many_loops (unsigned loops) {
	/* Wait for a timer tick. */
	int64_t start = ticks;
	while (ticks == start)
		barrier ();

	/* Run LOOPS loops. */
	start = ticks;
	busy_wait (loops);

	/* If the tick count changed, we iterated too long. */
	barrier ();
	return start != ticks;
}

/* Iterates through a simple loop LOOPS times, for implementing
   brief delays.

   Marked NO_INLINE because code alignment can significantly
   affect timings, so that if this function was inlined
   differently in different places the results would be difficult
   to predict. */
static void NO_INLINE
busy_wait (int64_t loops) {
	while (loops-- > 0)
		barrier ();
}

/* Sleep for approximately NUM/DENOM seconds. */
static void
real_time_sleep (int64_t num, int32_t denom) {
	/* Convert NUM/DENOM seconds into timer ticks, rounding down.

	   (NUM / DENOM) s
	   ---------------------- = NUM * TIMER_FREQ / DENOM ticks.
	   1 s / TIMER_FREQ ticks
	   */
	int64_t ticks = num * TIMER_FREQ / denom;

	ASSERT (intr_get_level () == INTR_ON);
	if (ticks > 0) {
		/* We're waiting for at least one full timer tick.  Use
		   timer_sleep() because it will yield the CPU to other
		   processes. */
		timer_sleep (ticks);
	} else {
		/* Otherwise, use a busy-wait loop for more accurate
		   sub-tick timing.  We scale the numerator and denominator
		   down by 1000 to avoid the possibility of overflow. */
		ASSERT (denom % 1000 == 0);
		busy_wait (loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000));
	}
}
