import logging
from datetime import datetime
import os

class logger():

    def __init__(self, name, filename="./test.log"):
        self.name = name
        self.filename = filename
        self.savedir = os.path.dirname(filename)
        pass
    
    def emit_colored_ansi(self, fn):
        def new(*args):
            levelno = args[1].levelno
            if(levelno >= 50):
                color = '\x1b[31m'  # red
            elif(levelno >= 40):
                color = '\x1b[31m'  # red
            elif(levelno >= 30):
                color = '\x1b[33m'  # yellow
            elif(levelno >= 20):
                color = '\x1b[32m'  # green
            elif(levelno >= 10):
                color = '\x1b[35m'  # pink
            else:
                color = '\x1b[0m'  # normal
            args[1].levelname = color + args[1].levelname + '\x1b[0m'  # normal
            return fn(*args)
        return new
    
    def setup_logger(self):
        logger = logging.getLogger(self.name)
        logger.setLevel(logging.DEBUG)
        #log_format = logging.Formatter("%(asctime)-s: [%(levelname)s: %(filename)s:%(lineno)s - %(funcName)s() ] %(message)s")
        log_format = logging.Formatter("%(asctime)-s: [%(levelname)s: %(filename)s:%(lineno)s] %(message)s")
    
        fh = logging.FileHandler(self.filename)
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(log_format)
    
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        ch.setFormatter(log_format)
        logging.StreamHandler.emit = self.emit_colored_ansi(logging.StreamHandler.emit)

        logger.addHandler(fh)
        logger.addHandler(ch)
        return logger

    def obslog(self, log, filename="obs.log", lv=0):
        now = datetime.utcnow()
        str_t = now.strftime("%Y-%m-%d %H:%M:%S")
        if isinstance(log, list):
            log = " ".join(log)
        f = open(os.path.join(self.savedir, "{}_obs.txt".format(now.strftime("%Y%m%d"))), "a")
        if lv == 0:
            f.write("- [(UTC) {}] {}".format(str_t, log) + "\n")
        if lv == 1:
            f.write("    - [(UTC) {}] {}".format(str_t, log) + "\n")
        f.close()

if __name__ == "__main__":
    logger = logger(__name__)
    pass
