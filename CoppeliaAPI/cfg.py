# Author: Alberto Martínez Rodríguez
# Date: January 2022

import configparser
from threading import RLock

csimLock= RLock()

class Cfg:
    def __init__(self):
        parser = configparser.ConfigParser()
        parser.read('./.defaultConfig.ini')
        parser.read('./config.ini')
        parser.read('./CoppeliaAPI/.defaultConfig.ini')
        parser.read('./CoppeliaAPI/config.ini')
        print(parser.sections())
        self.config = parser["CUSTOM"]


    def __getattr__(self, name):
        """ parses first time """
        if name in self.config:
            if self.config[name] in ('True', 'False'):
                return self.config.getboolean(name)
            elif self.config[name].isdigit():
                return self.config.getint(name) 
            else:
                return self.config[name]
        else:
            # bad argument
            raise AttributeError("config file doesn't contains " + name)