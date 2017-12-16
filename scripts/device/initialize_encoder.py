import pyinterface

board_name = 6204
rsw_id = 0

print("board : CPZ-6204")
print("rsw : 1")
print("\n")
yn = input("initialize OK??(Y/n) : ")

if yn == "Y":
    print("initialize start")
    self.dio = pyinterface.open(board_name, rsw_id)
    self.dio.initialize()
    self.dio.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=1)
    self.dio.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=2)
    self.dio.set_z_mode(clear_condition="CLS0", latch_condition="", z_polarity=0, ch=1)
    self.dio.set_z_mode(clear_condition="CLS0", latch_condition="", z_polarity=0, ch=2)
    print("initialize end")
else:
    print("no initialize, ok")
    print("see you")

    
