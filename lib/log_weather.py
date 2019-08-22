import csv

header_name = ["intemp", "outtemp", "inhumi", "outhumi", "winddir", "windsp",
               "press", "rain", "cabin_temp1", "cabin_temp2", "dome_temp1",
               "dome_temp2", "gen_temp1", "gen_temp2"]

class Weather_log():
    def __init__(self, path):
        self.path = path

    def initialize(self):
        with open(self.path, "w") as f:
            d_names = ["intemp", "outtemp"]
            writer = csv.DictWriter(f, fieldnames=header_name)
            writer.writeheader()

    def write(self, intemp, outtemp, inhumi, outhumi, winddir, windsp, press, rain,
              cabin_temp1, cabin_temp2, dome_temp1, dome_temp2, gen_temp1, gen_temp2):
        with open(self.path, "a") as f:
            writer = csv.writer(f)
            writer.writerow([intemp, outtemp, inhumi, outhumi, winddir, windsp,
                             press, rain, cabin_temp1, cabin_temp2, dome_temp1, dome_temp2,
                             gen_temp1, gen_temp2])

    def read(self):
        with open(self.path, "r") as f:
            row = csv.DictReader(f)
            d = [i for i in row]
        return d
