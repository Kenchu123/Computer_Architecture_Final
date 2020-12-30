def trans(s):
    a = s.replace('_', '')
    b = int(a, 2)
    c = b.to_bytes(8, 'little')
    for i in c:
        print(hex(i), end=' ')

d = input()
trans(d)


