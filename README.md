# \# FC0917H6TEST

# 

# 這是一個基於 Python 開發的測試程式，主要用於與硬幣驗鈔設備透過 \*\*ccTalk 通訊協定\*\* 進行測試。

# 

# \## 功能

# \- 與設備建立串口連線 (COM Port)  

# \- 送出測試命令 (例如 0xFD Address Poll、0xF2 Request Serial Number 等)  

# \- 接收並解析設備回應  

# \- 提供簡易的測試流程（如 Intelligent Payout、Multiple Payment 等）  

# 

# ---

# 

# \## 環境需求

# \- Python 3.9 以上

# \- Windows 10/11 (其他系統需調整)

# \- 需要安裝的套件：

# &nbsp; ```bash

# &nbsp; pip install pyserial notebook

# 

