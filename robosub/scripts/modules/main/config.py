import ConfigParser
from shutil import copyfile


"""Reads and writes from config.ini"""

CONFIG_TEMPLATE_PATH = 'config/template_config.ini'
CONFIG_FILE_PATH = 'config/config.ini'
config = ConfigParser.RawConfigParser()


def read_file():
    """
    Opens the config.ini file
    Creates config.ini file from template_config.ini file if config.ini does not exist
    """

    try:
        config.readfp(open(CONFIG_FILE_PATH))
    except IOError:
        print('setting up config.ini file.')
        copyfile(CONFIG_TEMPLATE_PATH, CONFIG_FILE_PATH)
        config.readfp(open(CONFIG_FILE_PATH))


def string_parser(string):
    """ Parses the string into a list, bool, int, float, or string"""

    # converts string into a list
    if ', ' in string:
        config = []
        # converts each item in the list into its respective types
        for item in string.split(', '):
            config.append(string_parser(item))
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


def get_config(section, option):
    """
    Reads variables from config/config.ini file
    section -- (auv, cv)
    option -- variable name
    """

    read_file()

    if config.has_option(section, option):
        return string_parser(config.get(section, option))


def set_config(section, option, value):
    """
    Writes variables to config/config.ini file
    section -- name of section
    option -- name of variable
    value -- str (if sending list: convert value to string separated by ', ')
    """

    if not config.has_section(section):
        config.add_section(section)

    config.set(section, option, value)

    with open(CONFIG_FILE_PATH, 'wb') as configfile:
        config.write(configfile)


def reset_option(section, option):
    """
    Resets variable to default values if default_[option] exists
    section -- name of section
    option -- name of variable
    """

    if (config.has_section(section) and config.has_option(section, option) and
            config.has_option(section, 'default_' + option)):
        default = get_config(section, 'default_' + option)

        if isinstance(default, list):
            default = [str(item) for item in default]
            set_config(section, option, ', '.join(default))
        else:
            set_config(section, option, get_config(default))


def list_options(section):
    """
    Returns a list of all the options of the given section
    section -- name of section
    """

    options = []

    for option in config.items(section):
        options.append(option[0])

    return options
