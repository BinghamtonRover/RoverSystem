import json
import logging
import logging.handlers
import time

class Test:
    def __init__(self, name):
        self.name = name
        
    def toJSON(self):
        #Returns a string in JSON format
        #When passed into the Logger it prints to file
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

    def createLogger(self, fileName):
        #Creating and setting up the logger
        logger = logging.getLogger("Timed Logger")
        logger.setLevel(logging.INFO)
        handler = logging.handlers.TimedRotatingFileHandler(fileName, when="s", interval = 1, backupCount=5)
        logger.addHandler(handler)
              
        #This is me adding to the logger
        #I update an element (name) to check if it
        #correctly creates updated files
        for i in range(5000):
            if(i % 11 == 1):
                self.name += str(i)
            else:
                self.name += 'a'
            self.age = i
            logger.info(self.toJSON())
