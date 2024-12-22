##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
## Copyright (C) 2012-2014 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2022 DreamSourceLab <support@dreamsourcelab.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

## 开发脚本的前提条件是必须掌握python语言
## 更复杂的协议请查考(0-i2c、0-spi、 0-uart、1-i2c、1-spi、1-uart)

## text block fill color table:
##  [#EF2929,#F66A32,#FCAE3E,#FBCA47,#FCE94F,#CDF040,#8AE234,#4EDC44,#55D795,#64D1D2
##  ,#729FCF,#D476C4,#9D79B9,#AD7FA8,#C2629B,#D7476F]
 
# 导出核心模块类，c代码实现的类
import sigrokdecode as srd

# 协议模块类
class Decoder(srd.Decoder):
    # 这里定义类的一些全局变量，有一些是底层框架要求必须要写的，其它提根据需要
    # 自己加，注意缩进，不清楚请查看python手册

    # 说明需要安装的python版本
    api_version = 3

    # 协议标识，必须唯一，这里我们用"example"给协议命名
    id = 'mbi5264'

    # 协议名称, 不一定要求跟标识一致
    name = 'mbi5264'

    # 协议长名称
    longname = 'mbi5264'

    # 简介内容
    desc = 'This is an protocol for mbi5264'

    # 开源协议
    license = 'gplv2+'

    # 接收的输入的数据源名，如果是多层协议一起工作，可使用上一个协议的输出名
    inputs = ['logic']

    # 输出的数据源名，多层协议模式下，可作为下层协议的输入数据源名
    outputs = ['mbi5264']

    # 适用范围标签
    tags = ['Embedded/industrial']

    # 必须要绑定的通道定义，将在界面上可见
    # id:通道标识, 任意命名
    # type:类型，根据需要设置一个值, -1:COMMON,0:SCLK,1:SDATA,2:ADATA
    # name:标签名
    # desc:该通道的说明
    # 注意元组的最后的逗号不能少
    channels = (
        {'id': 'c1', type:0, 'name': 'dclk', 'desc': 'dclk'},
        {'id': 'c2', type:0, 'name': 'le', 'desc': 'le'},
        {'id': 'c3', type:1, 'name': 'di', 'desc': 'di'},
    )

    # 可选通道，其它跟上面的一样
    optional_channels = ()

    # 提供给用户通过界面设置的参数，根据业务需要来定义
    # 通过self.options[id]取值，id就是各个项的id值，比如下面的"wordsize"
    options = ()

    # 解析结果项定义
    # annotations里的每一项可以有2到3个属性，当有３个属性时，第一个表示类型
    # 类型对应0-16个颜色，当类型范围在200-299时，将绘制边沿箭头
    annotations = (
        ('1', 'cmd', 'command'),
        ('2', 'value', 'data value'),
    )

    # 解析结果行定义
    annotation_rows = (
        # (0,)表示可输出第1个定义的annotations类型
        ('cmd', 'row1', (0,)),

        # (1,2)表示可输出第1个和第2个定义的annotations类型
        ('value', 'row2', (1,)),
    )

    # 构造函数，自动被调用
    def __init__(self):
         # 这里调用一个类成员函数，完成一些参数的初始化
        self.reset()

    # 重置函数，在这里做一些重置和定义类私有变量工作
    def reset(self):
        # 定义一个私有变量count
        self.count = 0

    # 开始执行解码任务时，由c底层代码自动调用一次
    # 这里，完成一些解码结果项annotation类型的注册
    # 类型有: OUTPUT_ANN，OUTPUT_PYTHON，OUTPUT_BINARY，OUTPUT_META
    # self.register函数是c底层类提供的
    def start(self):
       self.out_ann = self.register(srd.OUTPUT_ANN)

    # 定义一个输出函数
    # a,b为采样位置的起点和终点
    # ann为annotations定义的项序号
    # data是一个列表，列表里有１到３个字符串，它们将显示到屏幕
    # annotation输出到哪一行由annotation_rows决定
    # self.out_ann就是上面注册的消息类型了
    # self.put是c底层类提供的函数
    def put_ann(self, a, b, ann, data):
        self.put(a, b, self.out_ann, [ann, data])

    # 解码函数，解码任务开始时由c底层代码调用
    # 这里不断循环等待所有采样数据被处理完成
    # 下面的示例代码是解析某一通道的数据，从向上边沿开始到向下边沿结束，输出它们的样品位置差值，
    # 奇数次显示第二行，偶数次显示在第一行，我们只指定annotations里定义的序号
    # 软件会自动根据annotation_rows的设置，决定显示在哪一行
    def decode(self):
        value = 0
        cmd = 0
        last_clk = 0
        last_le = 0
        clks = 0
        show_width = 160

        # 不断循环，处理完所有数据
        while True:
            # 取一个条件，调用wait函数，找到符合条件的数据
            # (a,b)是一个元组，里边的变量数一定要跟channel数一致    
            (clk, le, dio) = self.wait()
            if clk != last_clk:
                clks += 1
                value <<= 1
                value &= 0xffff
                value |= dio
                if le > 0:
                    cmd += 1
            if clks == 16 and show_width != 160:
                show_width = self.samplenum
            # le下降锁存命令
            if le == 0 and last_le > 0:
                start = self.samplenum - show_width if self.samplenum > show_width else 0
                end = self.samplenum
                row1 = '%02X' % cmd
                row2 = '%02X' % value
                self.put_ann(start, end, 0, [row1])
                self.put_ann(start, end, 1, [row2])
            if le == 0:
                cmd = 0
            last_clk = clk
            last_le = le

