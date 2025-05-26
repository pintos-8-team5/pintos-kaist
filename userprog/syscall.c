#include "userprog/syscall.h"
#include <stdio.h>
#include <syscall-nr.h>
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/loader.h"
#include "userprog/gdt.h"
#include "threads/flags.h"
#include "intrinsic.h"
#include <stdbool.h>
#include "filesys/filesys.h"  
#include "threads/palloc.h"


void syscall_entry (void);
void syscall_handler (struct intr_frame *);

/*----------------check----------------*/
void check_address(void *addr);
/*----------------check----------------*/

/*----------------handler----------------*/
void halt(void);
void exit(int status);
int write(int fd, const void *buffer, unsigned size);
bool create(const char *file, unsigned initial_size);
bool remove(const char *file);
int open(const char *file);
// int add_file_to_fdt(struct file *file);
int filesize(int fd);
static struct file *find_file_by_fd(int fd);
void close(int fd);
unsigned tell(int fd);
void seek(int fd, unsigned position);
int read(int fd, void *buffer, unsigned size);
/*----------------handler----------------*/

/* System call.
 *
 * Previously system call services was handled by the interrupt handler
 * (e.g. int 0x80 in linux). However, in x86-64, the manufacturer supplies
 * efficient path for requesting the system call, the `syscall` instruction.
 *
 * The syscall instruction works by reading the values from the the Model
 * Specific Register (MSR). For the details, see the manual. */

#define MSR_STAR 0xc0000081         /* Segment selector msr */
#define MSR_LSTAR 0xc0000082        /* Long mode SYSCALL target */
#define MSR_SYSCALL_MASK 0xc0000084 /* Mask for the eflags */

void syscall_init(void) {
	write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48  |
			((uint64_t)SEL_KCSEG) << 32);
	write_msr(MSR_LSTAR, (uint64_t) syscall_entry);

/* The interrupt service rountine should not serve any interrupts
	* until the syscall_entry swaps the userland stack to the kernel
	* mode stack. Therefore, we masked the FLAG_FL. */
	write_msr(MSR_SYSCALL_MASK,
			FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);
}

/* The main system call interface */
void
syscall_handler (struct intr_frame *f UNUSED) {

	int sys_number = f->R.rax;
	switch (sys_number)
	{ // rax is the system call number
	case SYS_HALT:
		halt(); // pintos를 종료시키는 시스템 콜
		break;
	case SYS_EXIT:
		exit(f->R.rdi); // 현재 프로세스를 종료시키는 시스템 콜
		break;
	// case SYS_FORK:
	// 	f->R.rax = fork(f->R.rdi, f);
	// 	break;
	// case SYS_EXEC:
	// 	if (exec(f->R.rdi) == -1)
	// 	{
	// 		exit(-1);
	// 	}
	// 	break;
	// case SYS_WAIT:
	// 	f->R.rax = process_wait(f->R.rdi);
	// 	break;
	case SYS_CREATE:
		f->R.rax = create((const char *)f->R.rdi, (unsigned)f->R.rsi);
		break; 
	case SYS_REMOVE:
		f->R.rax = remove(f->R.rdi);
		break;
	case SYS_OPEN:
		f->R.rax = open(f->R.rdi);
		break;
	case SYS_FILESIZE:
		f->R.rax = filesize(f->R.rdi);
		break;
	case SYS_READ:
		f->R.rax = read(f->R.rdi, f->R.rsi, f->R.rdx);
		break;
	case SYS_WRITE:
		f->R.rax = write(f->R.rdi, f->R.rsi, f->R.rdx);
		break;
	case SYS_SEEK:
		seek(f->R.rdi, f->R.rsi);
		break;
	case SYS_TELL:
		f->R.rax = tell(f->R.rdi);
		break;
	case SYS_CLOSE:
		close(f->R.rdi);
		break;
	default:
		exit(-1);
		break;
	}
}

/*----------------handler----------------*/

void exit(int status)
{
	struct thread *t = thread_current();
    t->exit_status = status;
    printf("%s: exit(%d)\n", t->name, t->exit_status); 
    thread_exit();
}

void halt(void)
{
	power_off();
}

int write(int fd, const void *buffer, unsigned size)
{
	check_address(buffer);

	int result;

	if (fd < 0 || fd >= FDCOUNT_LIMIT)
	{
		return -1;
	}

	if (fd == 1)
	{
		putbuf(buffer, size);
		result = size;
	}

	else if (fd == 0)
	{
		result = -1;
	}

	else if (fd >= 2)
	{
		struct file *f = thread_current()->fd_table[fd];
		if (f == NULL)
		{
			return -1;
		}

		result = file_write(f, buffer, size);
	}

	else
	{
		result = -1;
	}

	return result;
}

int read(int fd, void *buffer, unsigned size)
{
	check_address(buffer);

	int bytes_read = 0;

	if (fd < 0 || fd >= FDCOUNT_LIMIT)
	{
		return -1;
	}

	if (fd == 0)
	{

		for (int i = 0; i < size; i++)
		{
			uint8_t *buf = (uint8_t *)buffer;
			buf[i] = input_getc();
		}
		bytes_read = size;
	}

	else if (fd == 1)
	{
		return -1;
	}

	else if (fd >= 2)
	{
		struct file *f = thread_current()->fd_table[fd];
		if (f == NULL)
		{
			return -1;
		}

		bytes_read = file_read(f, buffer, size);
	}

	return bytes_read;
}

// int read(int fd, void *buffer, unsigned size)
// {
// 	check_address(buffer);

// 	int read_result;
// 	struct thread *cur = thread_current();
// 	struct file *file_fd = find_file_by_fd(fd);

// 	if (fd == 0)
// 	{
// 		// read_result = i;
// 		*(char *)buffer = input_getc(); // 키보드로 입력 받은 문자를 반환하는 함수
// 		read_result = size;
// 	}
// 	else
// 	{
// 		if (find_file_by_fd(fd) == NULL)
// 		{
// 			return -1;
// 		}
// 		else
// 		{
// 			lock_acquire(&filesys_lock);
// 			read_result = file_read(find_file_by_fd(fd), buffer, size);
// 			lock_release(&filesys_lock);
// 		}
// 	}
// 	return read_result;
// }

// int write(int fd, const void *buffer, unsigned size)
// {
// 	check_address(buffer);

// 	int write_result;
// 	lock_acquire(&filesys_lock);
// 	if (fd == 1)
// 	{
// 		putbuf(buffer, size); // 문자열을 화면에 출력하는 함수
// 		write_result = size;
// 	}
// 	else if (fd == 0)
// 	{
// 		write_result = -1;
// 	}

// 	else if (fd >= 2)
// 	{
// 		write_result = file_write(find_file_by_fd(fd), buffer, size);
// 	}
// 	lock_release(&filesys_lock);
// 	return write_result;
// }

bool create(const char *file, unsigned initial_size)
{
	check_address(file);
	return filesys_create(file, initial_size);
}
 
bool remove(const char *file)
{
	check_address(file);

	return filesys_remove(file);
}

int open(const char *file)
{
	check_address(file);
	struct file *f = filesys_open(file);
	if (f == NULL)
	{
		return -1;
	}
	struct thread *t = thread_current();
	int fd = t->fd_idx++;
	t->fd_table[fd] = f;
	return fd;
}

// int add_file_to_fdt(struct file *file)
// {
// 	struct thread *cur = thread_current();
// 	struct file **fdt = cur->fd_table;

// 	// fd의 위치가 제한 범위를 넘지 않고, fdtable의 인덱스 위치와 일치한다면
// 	while (cur->fd_idx < FDCOUNT_LIMIT && fdt[cur->fd_idx])
// 	{
// 		cur->fd_idx++;
// 	}

// 	// fdt이 가득 찼다면
// 	if (cur->fd_idx >= FDCOUNT_LIMIT)
// 		return -1;

// 	fdt[cur->fd_idx] = file;
// 	return cur->fd_idx;
// }

int filesize(int fd)
{
	struct file *open_file = find_file_by_fd(fd);
	if (open_file == NULL)
	{
		return -1;
	}
	return file_length(open_file);
}

static struct file *find_file_by_fd(int fd)
{
	struct thread *cur = thread_current();
	return cur->fd_table[fd];
}

void seek(int fd, unsigned position)
{
	struct file *file = find_file_by_fd(fd);
	if (file)
		file_seek(file, position);
}

unsigned tell(int fd)
{
	struct file *file = find_file_by_fd(fd);
	if (file)
		return file_tell(file);
	return -1;
}

void close(int fd)
{
	struct thread *cur = thread_current();
	if (fd < 0 || fd >= FDCOUNT_LIMIT || !cur->fd_table[fd])
		return;

	file_close(cur->fd_table[fd]);
	cur->fd_table[fd] = NULL;
}

/*----------------handler----------------*/

/*----------------check----------------*/
void check_address(void *addr)
{
	struct thread *cur = thread_current();
	if (addr == NULL || !(is_user_vaddr(addr)) || pml4_get_page(cur->pml4, addr) == NULL)
	{
		exit(-1);
	}
}

/*----------------check----------------*/