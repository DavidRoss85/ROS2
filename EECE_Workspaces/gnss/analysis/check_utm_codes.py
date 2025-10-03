import utm

class PrintUTM:
    
    def __init__(self,lat,lon):
        self.__lat = lat
        self.__lon = lon
        self.__utmE = 0
        self.__utmN = 0
        self.__zone = 0
        self.__letter = ''
        utm_values = utm.from_latlon(self.__lat,self.__lon) #Returns a tuple with UTM information
        self.__utmE, self.__utmN, self.__zone, self.__letter = utm_values #Unpack UTM information

        print(f"UTME: {self.__utmE}\nUTMN: {self.__utmN}")


def main():
    coords = PrintUTM(47.524771,-122.369952)


main()