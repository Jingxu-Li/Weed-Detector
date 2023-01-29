import logging
from logging.handlers import TimedRotatingFileHandler

logger = logging.getLogger()

# create console handler with a higher log level
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# create formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# create log time split handler and add it to the handlers
log_time_handler = TimedRotatingFileHandler('sprayer.log', when='M')
ch.setFormatter(formatter)
# add the handlers to the logger
logger.addHandler(ch)
logger.addHandler(log_time_handler)