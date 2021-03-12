QUOTE_MINIMAL = ';'


class WriterMock:
    def writerow(self, data):
        print(data)


def writer(log_file, delimiter=',', quotechar='|', quoting=';'):
    return WriterMock()
