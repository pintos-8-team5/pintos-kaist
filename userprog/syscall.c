#define USERPROG
#include "userprog/syscall.h"
#include "userprog/process.h"
#include <stdio.h>
#include <syscall-nr.h>
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/loader.h"
#include "userprog/gdt.h"
#include "threads/flags.h"
#include "intrinsic.h"
#include "filesys/filesys.h"
#include "filesys/file.h"
#include "userprog/exception.h"
#include "threads/palloc.h"
#include "threads/init.h"
#include "devices/input.h"

void syscall_entry(void);
void syscall_handler(struct intr_frame *);
bool sys_create(const char *file, unsigned initial_size);
int sys_open(const char *file);
int sys_write(int fd, const void *buffer, unsigned size);
int sys_read(int fd, void *buffer, unsigned size);
void sys_exit(int status);
void sys_halt(void);

/* System call.
 *
 * Previously system call services was handled by the interrupt handler
 * (e.g. int 0x80 in linux). However, in x86-64, the manufacturer supplies
 * efficient path for requesting the system call, the `syscall` instruction.
 *
 * The syscall instruction works by reading the values from the the Model
 * Specific Register (MSR). For the details, see the manual. */

#define MSR_STAR 0xc0000081			/* Segment selector msr */
#define MSR_LSTAR 0xc0000082		/* Long mode SYSCALL target */
#define MSR_SYSCALL_MASK 0xc0000084 /* Mask for the eflags */

// void syscall_init(void)
// {
// 	write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48 |
// 							((uint64_t)SEL_KCSEG) << 32);
// 	write_msr(MSR_LSTAR, (uint64_t)syscall_entry);

// 	/* The interrupt service rountine should not serve any interrupts
// 	 * until the syscall_entry swaps the userland stack to the kernel
// 	 * mode stack. Therefore, we masked the FLAG_FL. */
// 	write_msr(MSR_SYSCALL_MASK,
// 			  FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);
// }

// static int64_t get_user(const uint8_t *uaddr)
// {
// 	int64_t result;
// 	__asm __volatile(
// 		"movabsq $done_get, %0\n"
// 		"movzbq %1, %0\n"
// 		"done_get:\n"
// 		: "=&a"(result) : "m"(*uaddr));
// 	return result;
// }

// void handle_write(struct intr_frame *f)
// {
// }

// /* The main system call interface */
// void syscall_handler(struct intr_frame *f UNUSED)
// {
// 	// TODO: Your implementation goes here.

// 	int syscall_number = f->R.rax;
// 	int arg1 = f->rsp + 4;

// 	switch (syscall_number)
// 	{
// 	case SYS_READ:

// 	case SYS_WRITE:
// 		handle_write(f);
// 		break;
// 	}

// 	printf("system call!\n");
// 	thread_exit();
// }

void syscall_init(void)
{
	write_msr(MSR_STAR,
			  ((uint64_t)SEL_UCSEG - 0x10) << 48 | ((uint64_t)SEL_KCSEG) << 32);
	write_msr(MSR_LSTAR, (uint64_t)syscall_entry);

	/* The interrupt service rountine should not serve any interrupts
	 * until the syscall_entry swaps the userland stack to the kernel
	 * mode stack. Therefore, we masked the FLAG_FL. */
	write_msr(MSR_SYSCALL_MASK,
			  FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);
}

void syscall_handler(struct intr_frame *f)
{
	uint64_t syscall_num = f->R.rax;

	switch (syscall_num)
	{
	case SYS_CREATE:
		f->R.rax = sys_create((const char *)f->R.rdi, (unsigned)f->R.rsi);
		break;

	case SYS_OPEN:
		f->R.rax = sys_open((const char *)f->R.rdi);
		break;

	case SYS_FILESIZE:
		f->R.rax = sys_filesize((int)f->R.rdi);
		break;

	case SYS_WRITE:
		f->R.rax = sys_write((int)f->R.rdi, (const void *)f->R.rsi, (unsigned)f->R.rdx);
		break;

	case SYS_READ:
		f->R.rax = sys_read((int)f->R.rdi, (void *)f->R.rsi, (unsigned)f->R.rdx);
		break;

	case SYS_CLOSE:
		sys_close((int)f->R.rdi);
		break;

	case SYS_EXIT:
		sys_exit(f->R.rdi);
		break;

	case SYS_HALT:
		sys_halt();
		break;

	case SYS_FORK:
		f->R.rax = sys_fork((const char *)f->R.rdi, f);
		break;

	case SYS_WAIT:
		f->R.rax = sys_wait(f->R.rdi);
		break;

	case SYS_EXEC:
		f->R.rax = sys_exec(f->R.rdi);
		break;

	case SYS_SEEK:
		sys_seek(f->R.rdi, f->R.rsi);
		break;

	default:
		printf("Unknown syscall number: %lld\n", syscall_num);
		thread_exit();
	}
}

void check_address(void *addr)
{
	struct thread *t = thread_current();

	if (!is_user_vaddr(addr) || addr == NULL || pml4_get_page(t->pml4, addr) == NULL)
	{
		sys_exit(-1);
	}
}

bool sys_create(const char *file, unsigned initial_size)
{
	check_address(file);

	return filesys_create(file, initial_size);
}

int sys_open(const char *file)
{
	check_address(file);

	struct file *f = filesys_open(file);
	if (f == NULL)
	{
		return -1;
	}

	struct thread *t = thread_current();
	int fd = t->next_fd++;
	t->fd_table[fd] = f;
	return fd;
}

int sys_filesize(int fd)
{
	struct file *f = thread_current()->fd_table[fd];
	if (f == NULL)
	{
		return -1;
	}
	return file_length(f);
}

int sys_write(int fd, const void *buffer, unsigned size)
{
	check_address(buffer);

	int result;

	if (fd < 0 || fd >= FD_MAX)
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

int sys_read(int fd, void *buffer, unsigned size)
{
	check_address(buffer);

	int bytes_read = 0;

	if (fd < 0 || fd >= FD_MAX)
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

void sys_close(int fd)
{
	if (fd < 2 || fd >= FD_MAX)
	{
		return;
	}

	struct file *f = thread_current()->fd_table[fd];
	if (f == NULL)
	{
		return;
	}
	file_close(f);
	thread_current()->fd_table[fd] = NULL;
}

void sys_exit(int status)
{
	struct thread *curr = thread_current();
	curr->exit_status = status;
	printf("%s: exit(%d)\n", thread_name(), status);

	thread_exit();
}

void sys_halt(void)
{
	power_off();
}

tid_t sys_fork(const char *thread_name, struct intr_frame *f)
{
	return process_fork(thread_name, f);
}

int sys_wait(tid_t pid)
{
	return process_wait(pid);
}

int sys_exec(const char *cmd_line)
{
	check_address(cmd_line);

	char *cmd_line_copy;
	cmd_line_copy = palloc_get_page(0);
	if (cmd_line_copy == NULL)
	{
		sys_exit(-1);
	}
	strlcpy(cmd_line_copy, cmd_line, PGSIZE);

	if (process_exec(cmd_line_copy) == -1)
	{
		sys_exit(-1);
	}
}

void sys_seek(int fd, unsigned position)
{
	if (fd < 0 || fd >= FD_MAX)
	{
		return;
	}

	struct file *f = thread_current()->fd_table[fd];
	if (f == NULL)
	{
		return;
	}

	file_seek(f, position);
}
