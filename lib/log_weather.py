import csv
import pandas

header_name = ["timestamp", "intemp", "outtemp", "inhumi", "outhumi", "winddir", "windsp",
               "press", "rain", "cabin_temp1", "cabin_temp2", "dome_temp1",
               "dome_temp2", "gen_temp1", "gen_temp2", "scan_num", "obs_mode"]

class Weather_log():
    def __init__(self, path):
        self.path = path

    def initialize(self):
        with open(self.path, "w") as f:
            d_names = ["intemp", "outtemp"]
            writer = csv.DictWriter(f, fieldnames=header_name)
            writer.writeheader()

    def write(self, timestamp, intemp, outtemp, inhumi, outhumi, winddir, windsp, press, rain,
              cabin_temp1, cabin_temp2, dome_temp1, dome_temp2, gen_temp1, gen_temp2, scan_number="", obs_mode=""):
        with open(self.path, "a") as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, intemp, outtemp, inhumi, outhumi, winddir, windsp,
                             press, rain, cabin_temp1, cabin_temp2, dome_temp1, dome_temp2,
                             gen_temp1, gen_temp2, scan_number, obs_mode])

    def read(self):
        return pandas.read_csv(self.path)
