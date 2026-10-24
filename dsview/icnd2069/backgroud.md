261024_100m_20ms.dsl 是icnd2069用逻辑分析仪(dsview 应该是基于开源软件的)抓包的数据 
ch0 clk
ch1 lat
ch2 red_data
ch3 oe(换行) 
ch4 rt5990的某个pin
ch5 green_data
ch6 blue_data
ch7 rt5990的某个pin

dsview/mbi5264/pd.py 是我之前自己写的 dsview用的mbi5264的解码代码

现在希望你写一个rust程序到dsview/icnd2069 解码分析clk lat 以及rgb data组成的指令 输入是dsl文件和ch通道(有默认直) 输出是包含的所有指令和指令对应的数据
