from abc import ABC, abstractmethod

class Filter(ABC):
    def __init__(self):
        self.filters = None

    @abstractmethod
    def filter(self, data_input):
        """
        对输入数据进行滤波处理
        Args:
            data_input: 输入数据
        Returns:
            data_output: 滤波后的数据
        """
        pass

    def filter_multiple(self, data_input):
        """
        对输入数据集进行滤波处理
        Args:
            data_input: 输入数据集
        Returns:
            data_output: 滤波后的数据集
        """
        if self.filters is None:
            self.filters = [self.__class__() for _ in range(len(data_input))]
        return [self.filters[i].filter(data_input[i]) for i in range(len(data_input))]

class FirstOrderLowPassFilter(Filter):
    def __init__(self, alpha=0.5):
        super().__init__()
        self.alpha = alpha

        self.data_recoded = None

    def filter(self, data_input):
        if self.data_recoded is None:
            self.data_recoded = data_input
            return data_input

        data_output = self.alpha * self.data_recoded + (1 - self.alpha) * data_input
        self.data_recoded = data_output
        return data_output
    
if __name__ == "__main__":
    a = FirstOrderLowPassFilter(alpha=0.5)
    data_input = [1, 1.2, 1.1, 0.9, 1.02]
    for data in data_input:
        print(a.filter(data))

    print("_" * 100)

    b = FirstOrderLowPassFilter(alpha=0.5)
    data_set_input = [[1*i, 1.2*i, 1.1*i, 0.9*i, 1.02*i] for i in [1,1.2,1.01,1.11,0.96,0.95,1.03]]
    for data in data_set_input:
        print(b.filter_multiple(data))