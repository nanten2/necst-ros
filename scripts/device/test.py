def get_pos(bin, bin2):
    buff = []
    buff2 = []
                
    for i in range(8):
        if i != 0 and bin == 0:
            buff.append(0)
        if bin % 2 == 0:
            buff.append(0)
        else:
            buff.append(1)
        bin = int(bin/2)
        
    for i in range(8):
        if i != 0 and bin2 == 0:
            buff2.append(0)
        if bin2 % 2 == 0:
            buff2.append(0)
        else:
            buff2.append(1)
        bin2 = int(bin2/2)
        
    total = (buff[0]*1+buff[1]*2+buff[2]*pow(2.,2.)+buff[3]*pow(2.,3.))/100.0
    total = total + (buff[4]*1+buff[5]*2+buff[6]*pow(2.,2.)+buff[7]*pow(2.,3.))/10.0
    total2 = buff2[0]*1+buff2[1]*2+buff2[2]*pow(2.,2.)+buff2[3]*pow(2.,3.)
    total2 = total2 + (buff2[4]*1)*10
        
    m_pos = (total+total2)*pow(-1.,(buff2[5]+1))
    return m_pos

num = 0
_list = []
for i in range(154):
    dt = i%16
    if not 0 <= dt <= 5 or i < 15:
        _list.append(i)

for i in range(len(_list)):
    num = (495+(int(8000/256)+8000%256))*i
    if True:
        aa = get_pos(1,32)
        print(i, 0, " : ", aa)

