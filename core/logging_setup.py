import logging
import os


def setup_logging(runtime_dir, level):
    if not os.path.isdir(runtime_dir):
        os.makedirs(runtime_dir)
    log_path = os.path.join(runtime_dir, "captain.log")
    logger = logging.getLogger("nevc_auto")
    logger.setLevel(getattr(logging, level, logging.INFO))
    logger.handlers = []
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(name)s - %(message)s")
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    logger.addHandler(console)
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    return logger

