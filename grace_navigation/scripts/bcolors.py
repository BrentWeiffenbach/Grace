#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python


class bcolors:
    """A functionally useless class. But I thought I would have fun color coding the console outputs :)

    Source: https://stackoverflow.com/a/287944
    """

    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"

    @staticmethod
    def green(text: str) -> str:
        return f"{bcolors.OKGREEN}{text}{bcolors.ENDC}"

    @staticmethod
    def cyan(text: str) -> str:
        return f"{bcolors.OKCYAN}{text}{bcolors.ENDC}"

    @staticmethod
    def blue(text: str) -> str:
        return f"{bcolors.OKBLUE}{text}{bcolors.ENDC}"

    @staticmethod
    def header(text: str) -> str:
        return f"{bcolors.HEADER}{text}{bcolors.ENDC}"

    @staticmethod
    def warning(text: str) -> str:
        return f"{bcolors.WARNING}{text}{bcolors.ENDC}"

    @staticmethod
    def fail(text: str) -> str:
        return f"{bcolors.FAIL}{text}{bcolors.ENDC}"

    @staticmethod
    def bold(text: str) -> str:
        return f"{bcolors.BOLD}{text}{bcolors.ENDC}"

    @staticmethod
    def underline(text: str) -> str:
        return f"{bcolors.UNDERLINE}{text}{bcolors.ENDC}"
