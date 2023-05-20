// SPDX-FileCopyrightText: 2021 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#ifndef _WIN32

#include "fastjmp.h"

#if defined(__APPLE__)
#define PREFIX "_"
#else
#define PREFIX ""
#endif

asm("\t.global " PREFIX "fastjmp_set\n"
    "\t.global " PREFIX "fastjmp_jmp\n"
    "\t.text\n"
    "\t" PREFIX "fastjmp_set:"
    R"(
	movq 0(%rsp), %rax
	movq %rsp, %rdx			# fixup stack pointer, so it doesn't include the call to fastjmp_set
	addq $8, %rdx
	movq %rax, 0(%rdi)	# actually rip
	movq %rbx, 8(%rdi)
	movq %rdx, 16(%rdi)	# actually rsp
	movq %rbp, 24(%rdi)
	movq %r12, 32(%rdi)
	movq %r13, 40(%rdi)
	movq %r14, 48(%rdi)
	movq %r15, 56(%rdi)
	xorl %eax, %eax
	ret
)"
    "\t" PREFIX "fastjmp_jmp:"
    R"(
	movl %esi, %eax
	movq 0(%rdi), %rdx	# actually rip
	movq 8(%rdi), %rbx
	movq 16(%rdi), %rsp	# actually rsp
	movq 24(%rdi), %rbp
	movq 32(%rdi), %r12
	movq 40(%rdi), %r13
	movq 48(%rdi), %r14
	movq 56(%rdi), %r15
	jmp *%rdx
)");

#endif // __WIN32
