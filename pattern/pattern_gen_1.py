import random

class write_pattern(object):
    def __init__(self):
        self.counter = 0
        
    def write_line(self, str_in):
        assert(len(str_in) == 16)
        substr_0 = str_in[ 0: 2]
        substr_1 = str_in[ 2: 4]
        substr_2 = str_in[ 4: 6]
        substr_3 = str_in[ 6: 8]
        substr_4 = str_in[ 8:10]
        substr_5 = str_in[10:12]
        substr_6 = str_in[12:14]
        substr_7 = str_in[14:16]
        counter_str = '{:x}'.format(self.counter).zfill(2).upper()
        str_out = '{}_{}_{}_{}_{}_{}_{}_{} // '.format(substr_7, substr_6, substr_5, substr_4, substr_3, substr_2, substr_1, substr_0) + \
                  '{}_{}_{}_{}_{}_{}_{}_{} // '.format(substr_0, substr_1, substr_2, substr_3, substr_4, substr_5, substr_6, substr_7) + \
                  '0x{}//'.format(counter_str)
        self.counter = self.counter + 8
        return str_out

def add_underline(str_in):
    substr_0 = str_in[ 0: 2]
    substr_1 = str_in[ 2: 4]
    substr_2 = str_in[ 4: 6]
    substr_3 = str_in[ 6: 8]
    substr_4 = str_in[ 8:10]
    substr_5 = str_in[10:12]
    substr_6 = str_in[12:14]
    substr_7 = str_in[14:16]
    return '{}_{}_{}_{}_{}_{}_{}_{} (hex)'.format(substr_0, substr_1, substr_2, substr_3, substr_4, substr_5, substr_6, substr_7)

if __name__ == '__main__':
    # Modify your test pattern here
    x = random.randint(4294967296,8589934592)
    y = random.randint(1,10000)

    data1 = '{:x}'.format(x).zfill(16).upper()
    data2 = '{:x}'.format(y).zfill(16).upper()
    and_result = '{:x}'.format(x & y).zfill(16).upper()
    or_result = '{:x}'.format(x | y).zfill(16).upper()
    slt_result = '{:x}'.format(x < y).zfill(16).upper()
    add_result = '{:x}'.format(x + x).zfill(16).upper()
    sub_result = '{:x}'.format(2*x - x).zfill(16).upper()
    
    with open('data1.txt', 'w') as f_data:
        data = write_pattern()
        f_data.write(data.write_line(data1) + '\n')
        f_data.write(data.write_line(data2))

    with open('ans1.txt', 'w') as f_ans:
        ans = write_pattern()
        f_ans.write(ans.write_line(data1) + '\n')
        f_ans.write(ans.write_line(data2) + '\n')
        f_ans.write(ans.write_line(and_result) + '\n')
        f_ans.write(ans.write_line(or_result) + '\n')
        f_ans.write(ans.write_line(slt_result) + '\n')
        f_ans.write(ans.write_line(add_result) + '\n')
        f_ans.write(ans.write_line(sub_result))

    print('x      : ' + add_underline(data1))
    print('y      : ' + add_underline(data2))
    print('x & y  : ' + add_underline(and_result))
    print('x | y  : ' + add_underline(or_result))
    print('x < y  : ' + add_underline(slt_result))
    print('x + x  : ' + add_underline(add_result))
    print('2x - x : ' + add_underline(sub_result))