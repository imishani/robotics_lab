class KException(Exception):
    def __init__(self, description):
        self.description = description
        super().__init__(self.description)

    def __str__(self):
        return 'Error encountered description: {0}'.format(self.description)
