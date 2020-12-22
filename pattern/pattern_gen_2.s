.data
  num: .dword  0:22
.globl main


.text
main:
	li x1, 1
	li x2, 2
	li x3, 3
	li x4, 4
	li x5, 5
	li x6, 6
	addi x7, x1, 127
	xori x8, x7, 127
	slli x9, x2, 4
	sll x10, x9, x3
	sub x11, x10, x4
	xor x12, x11, x3
	and x13, x12, x8
	andi x14, x13, 6
	ori x15, x14, 249
	add x16, x7, x15
	srl x17, x16, x1
	or x18, x17, x16
	srli x19, x18, 1
	srai x20, x19, 3
	sra x21, x20, x1
	slti x21, x20, 32
	slt x22, x20, x9

	la x23, num
	sd x1, 0(x23)
	sd x2, 8(x23)
	sd x3, 16(x23)
	sd x4, 24(x23)
	sd x5, 32(x23)
	sd x6, 40(x23)
	sd x7, 48(x23)
	sd x8, 56(x23)
	sd x9, 64(x23)
	sd x10, 72(x23)
	sd x11, 80(x23)
	sd x12, 88(x23)
	sd x13, 96(x23)
	sd x14, 104(x23)
	sd x15, 112(x23)
	sd x16, 120(x23)
	sd x17, 128(x23)
	sd x18, 136(x23)
	sd x19, 144(x23)
	sd x20, 152(x23)
	sd x21, 168(x23)
	sd x22, 176(x23)
	