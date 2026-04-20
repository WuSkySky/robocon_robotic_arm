from abc import ABC, abstractmethod
from scipy.signal import butter

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
    
class SecondOrderButterworthLowPass(Filter):
    """二阶 Butterworth 低通，根据给定 Wn 自动计算系数"""
    def __init__(self, Wn=0.25):
        super().__init__()
        # 设计二阶 Butterworth 低通，获取 ba 系数
        b, a = butter(2, Wn, btype='low', output='ba')
        self.b = b.tolist()   # [b0, b1, b2]
        self.a = a.tolist()[1:] # [a1, a2]，a0=1 已归一化

        # 状态清零
        self.reset()

    def reset(self):
        self.x1 = 0.0
        self.x2 = 0.0
        self.y1 = 0.0
        self.y2 = 0.0

    def filter_sample(self, x_new):
        y_new = (
            self.b[0] * x_new +
            self.b[1] * self.x1 +
            self.b[2] * self.x2 -
            self.a[0] * self.y1 -
            self.a[1] * self.y2
        )
        self.x2 = self.x1
        self.x1 = x_new
        self.y2 = self.y1
        self.y1 = y_new
        return y_new

    def filter(self, data_input):
        # 同上，支持标量与序列
        if hasattr(data_input, '__len__'):
            return [self.filter_sample(x) for x in data_input]
        else:
            return self.filter_sample(data_input)
    
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