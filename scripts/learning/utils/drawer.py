import sys, os
import numpy as np
import plotly.graph_objects as go
import matplotlib
import matplotlib.pyplot as plt
from chart_studio.plotly import image as PlotlyImage
import chart_studio
from PIL import Image as PILImage
import io
from plotly.subplots import make_subplots
chart_studio.tools.set_credentials_file(username='Torinion', api_key='3gb8jfzZjocv11SQ4cYD')

class Drawer:
    def __init__(self):
        # self.existed_files = os.listdir(self.get_path("custom_logs"))
        self.existed_files = []

    def get_path(self, folder):
        path = os.path.dirname(os.path.realpath(__file__))
        path = "/".join(path.split("/")[:-1])
        path = os.path.join(path, folder)
        return path

    def analyse(self):
        files = os.listdir(self.get_path("custom_logs2"))
        not_drawn = list(set(files).difference(set(self.existed_files)))
        res = []
        for nd in not_drawn:
            temp = []
            for f in files:
                if nd in f:
                    temp.append(f)
            res.append(temp)
        return res

    def load(self, groups):
        arrays = {}
        for group in groups:
            array_group = []
            path = self.get_path("custom_logs2")
            for f in group:
                name = f[:-2]
                path_ = os.path.join(path, f)
                array_group.append(np.load(path_, allow_pickle=True))
            arrays[name] = np.array(array_group)
        return arrays

    def smooth(self, slice):
        W = 10
        slice_ = [slice[0]]
        for end in range(1, len(slice)+1):
            if end > W:
                start = end - W
            else:
                start = 0
            slice_.append(np.mean(slice[start:end]))
        # fig, ax = plt.subplots()
        # ax.plot(slice_)
        # plt.show()

        return slice_

    def preprocess(self, data):
        # Make arrays equally distanst
        # Transposing arrays
        keywords = ['reward', 'stability', 'projection']
        mean = {k: None for k in keywords}
        std = {k: None for k in keywords}

        data_ = []
        for array in data:
            data_.append(np.array(array).T)
        data = data_
        # Finding the shortest array
        min_ = 999999
        max_value = 0
        for array in data:
            if min_ > len(array[0]):
                min_ = len(array[0])
                max_value = array[0][-1]
        # New X axis
        STEP = 100
        X = [i for i in range(10, int(max_value), int(float(max_value)/min_))]
        # Make arrays equally distant
        arrays_ = {k: [] for k in keywords}
        for i in range(min_):
            y =  {k: [] for k in keywords}
            for j in range(len(data)):
                cnt = 0
                while X[i] > data[j][0][cnt]:
                    cnt += 1
                # It is important to not fuck up with 0 and 1 indexes
                y['reward'].append(data[j][1][cnt])
                y['stability'].append(data[j][2][cnt])
                y['projection'].append(data[j][3][cnt])
            for k in keywords:
                arrays_[k].append(y[k])
        for k in keywords:
            print('DBG smoothing')
            if k == 'projection':
                mean[k] = [np.mean([abs(1.-elem) for elem in frame]) for frame in arrays_[k]]
            else:
                mean[k] = [np.mean(frame) for frame in arrays_[k]]
            mean[k] = self.smooth(mean[k])
            std[k] = [np.std(frame) for frame in arrays_[k]]
            std[k] = self.smooth(std[k])
            arrays_[k].append(k)
        return X, mean, std

    def draw(self, name_, data):
        X, mean, std = self.preprocess(data)
        m = {'reward': 0, 'stability': 1, 'projection': 2}
        m_ = {0: 'reward', 1: 'stability', 2: 'projection'}
        dashs = [None, 'dash', 'dot']
        colors = ['rgb(180, 0, 0)', 'rgb(0, 180, 0)', 'rgb(0, 0, 180)']
        fillcolors = ['rgba(68, 40, 40, 0.2)', 'rgba(40, 68, 40, 0.2)', 'rgba(40, 40, 68, 0.2)']
        fig = make_subplots(rows=3, cols=1, x_title='Time steps',
                    y_title=name_,
                    subplot_titles=( [m_[i] for i in range(len(colors))]))
        for i in range(len(colors)):
            name = m_[i]
            number = m[name]

            trace = go.Scatter(
                name="name",
                x=X,
                y= self.smooth(mean[name]),
                showlegend=False,
                mode='lines',
                line=dict(color=colors[number], dash=dashs[number], width=3),
                fillcolor=fillcolors[number],
                fill='tonexty')
            fig.add_trace(trace, row=i+1, col=1)
            # fig = go.Figure(data=draw_data, layout=layout)
        fig.update_layout(showlegend=True, autosize=True)
        # path = self.get_path("images")
        # img_bytes = PlotlyImage.get(fig)
        # image = PILImage.open(io.BytesIO(img_bytes))
        # image.save(os.path.join(path, name_+".png"))
        fig.show()

    def load_manual(self, names):
        arrays = {name: [] for name in names}
        path = self.get_path("custom_logs2")
        for name in names:
            for f in os.listdir(path):
                if name in f:
                    path_ = os.path.join(path, f)
                    arrays[name].append(np.load(path_, allow_pickle=True))

        for k, v in arrays.items():
            print(k, np.array(v).shape)

        return arrays

    def draw_them_all(self, algs, data):
        m = {'reward': 0, 'stability': 1, 'projection': 2}
        m_ = {0: 'reward', 1: 'stability', 2: 'projection'}
        dashs = [None, 'dash', 'dot', 'solid']
        colors = ['rgb(180, 0, 0)', 'rgb(0, 180, 0)', 'rgb(0, 0, 180)', 'rgb(30, 30, 60)']
        fillcolors = ['rgba(68, 40, 40, 0.2)', 'rgba(40, 68, 40, 0.2)', 'rgba(40, 40, 68, 0.2)', 'rgba(20, 20, 20, 0.2)']
        for name, number in m.items():
            draw_data = []
            for i, alg in enumerate(algs):
                x = data[alg][0]
                mean = data[alg][1][name]
                std = data[alg][2][name]
                y = mean
                y_upper = [m + s for m, s in zip(mean, std)]
                y_lower = [m - s for m, s in zip(mean, std)]
                print('Number', i)
                upper_bound = go.Scatter(
                    x=x,
                    y=y_upper,
                    mode='lines',
                    # marker=dict(color="#111"),
                    line=dict(width=0),
                    showlegend=False,
                    fillcolor=fillcolors[i],
                    fill='tonexty')
                trace = go.Scatter(
                    name=alg.split('_')[0],
                    x=x,
                    y=y,
                    mode='lines',
                    line=dict(color=colors[i], dash=dashs[i], width=3),
                    fillcolor=fillcolors[i],
                    fill='tonexty')
                lower_bound = go.Scatter(
                    x=x,
                    showlegend=False,
                    y=y_lower,
                    mode='lines',
                    # marker=dict(color="#111"),
                    fillcolor=fillcolors[i],
                    line=dict(width=0)
                )
                draw_data.append(lower_bound)
                draw_data.append(trace)
                draw_data.append(upper_bound)
            layout = go.Layout(
                yaxis=dict(title=name.capitalize() + ' per episode'),
                xaxis=dict(title='Time steps'),
                showlegend=True)
            fig = go.Figure(data=draw_data, layout=layout)
            fig.show()
            # path = self.get_path("images")
            # img_bytes = PlotlyImage.get(fig)
            # image = PILImage.open(io.BytesIO(img_bytes))
            # image.save(os.path.join(path, '_'.join([a.split('_')[0] for a in algs])+"_"+name+".png"))
        # fig.show()

    def draw_multiple(self):
        files = os.listdir(self.get_path("custom_logs2"))
        names = list(set(["_".join(f.split("_")[:-1]) for f in files]))
        arrays_dict = self.load_manual(names)
        preprocessed = {k: self.preprocess(v) for k, v in arrays_dict.items()}
        print(preprocessed['PPO2_step_box_1_0_official_exps_1_nesm_no'])
        exit()
        self.draw_them_all(names, preprocessed)

    def draw_last(self):
        files_to_draw = self.analyse()
        arrays_dict = self.load(files_to_draw)
        print('To draw', files_to_draw)

        for k, v in arrays_dict.items():
            print('Drawing', k)
            self.draw(name_=k, data=v)

if __name__ == '__main__':
    d = Drawer()
    d.draw_multiple()
