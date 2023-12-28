## インストール方法
```
$ git clone https://github.com/TsutsumiAkinosuke/hairo23_ws.git
```

操縦UIを起動するにはPyQt5をインストールする必要があります
```
pip install PyQt5
```

## 使い方
```
$ cd hairo23_ws
$ . install/setup.bash
$ cd src
$ python3 operator_node.py
```

別のノードでJoyメッセージをパブリッシュすると押されたコントローラのボタンに反応してUI上のボタンの色が変わります  
ジョイスティックを動かすとUI上のダイアル(ジョイスティックを再現)が動きます
