import logging


def emit_colored_ansi(fn):
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
                                                        
def setup_logger(name, filename="./test.log"):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    #log_format = logging.Formatter("%(asctime)-s: [%(levelname)s: %(filename)s:%(lineno)s - %(funcName)s() ] %(message)s")
    log_format = logging.Formatter("%(asctime)-s: [%(levelname)s: %(filename)s:%(lineno)s] %(message)s")
    
    fh = logging.FileHandler(filename)
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(log_format)
    
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    ch.setFormatter(log_format)
    logging.StreamHandler.emit = emit_colored_ansi(logging.StreamHandler.emit)

    logger.addHandler(fh)
    logger.addHandler(ch)
    return logger

if __name__ == "__main__":
    logger = setup_logger(__name__)
    pass
