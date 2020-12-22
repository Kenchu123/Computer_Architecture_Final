.data
  num: .dword  0:7
.globl main


.text
main:
	li x1, 1
	li x2, 2
	li x3, 3
	li x4, 4
	li x5, 5
	li x6, 6
	sub x7, x1, x3

	la x23, num
	sd x1, 0(x23)
	sd x2, 8(x23)
	sd x3, 16(x23)
	sd x4, 24(x23)
	sd x5, 32(x23)
	sd x6, 40(x23)
	sd x7, 48(x23)
	
