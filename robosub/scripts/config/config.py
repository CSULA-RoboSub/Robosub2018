import ConfigParser
from shutil import copyfile


class Config():
    """Reads and writes from config.ini"""

    def __init__(self):
        self.CONFIG_TEMPLATE_PATH = 'config/template_config.ini'
        self.CONFIG_FILE_PATH = 'config/config.ini'
        self.config = ConfigParser.RawConfigParser()

        self.motor_state = None
        self.models = None

    def read_file(self):
        """
        Opens the config.ini file
        Creates config.ini file from template_config.ini file if config.ini does not exist
        """

        try:
            self.config.readfp(open(self.CONFIG_FILE_PATH))
        except IOError:
            print('setting up config.ini file.')
            copyfile(self.CONFIG_TEMPLATE_PATH, self.CONFIG_FILE_PATH)
            self.config.readfp(open(self.CONFIG_FILE_PATH))

    def string_parser(self, string):
        """ Parses the string into a list, bool, int, float, or string"""

        # converts string into a list
        if ', ' in string:
            config = []
            # converts each item in the list into its respective types
            for item in string.split(', '):
                config.append(self.string_parser(item))
            return config
        # converts string to boolean
        elif string == 'True':
            return True
        elif string == 'False':
            return False
        # converts string to int
        elif string.count('.') == 0:
            try:
                return int(string)
            except ValueError:
                pass
        # converts string to float
        else:
            try:
                return float(string)
            except ValueError:
                pass

        # does not convert string if already is a string
        return string

    def get_config(self, section, option):
        """
        Reads variables from config/config.ini file
        section -- (auv, cv)
        option -- variable name
        """

        self.read_file()

        if self.config.has_option(section, option):
            return self.string_parser(self.config.get(section, option))

    def set_config(self, section, option, value):
        """
        Writes variables to config/config.ini file
        section -- name of section
        option -- name of variable
        value -- str (if sending list: convert value to string separated by ', ')
        """

        if not self.config.has_section(section):
            self.config.add_section(section)

        self.config.set(section, option, value)

        with open(self.CONFIG_FILE_PATH, 'wb') as configfile:
            self.config.write(configfile)

    def reset_option(self, section, option):
        """
        Resets variable to default values if default_[option] exists
        section -- name of section
        option -- name of variable
        """

        if (self.config.has_section(section) and self.config.has_option(section, option) and
                self.config.has_option(section, 'default_' + option)):
            default = self.get_config(section, 'default_' + option)

            if isinstance(default, list):
                default = [str(item) for item in default]
                self.set_config(section, option, ', '.join(default))
            else:
                self.set_config(section, option, self.get_config(default))

    def list_options(self, section):
        """
        Returns a list of all the options of the given section
        section -- name of section
        """

        options = []

        for option in self.config.items(section):
            options.append(option[0])

        return options
