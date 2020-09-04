# 教師データの作成方法

教師データは２種類のファイルから成り立つ．

- 実験で作成した教師データのファイル `log_*.csv`
- 教師データの成否が記入されたファイル `status.csv`

この内，`status.csv`は実験中のメモをエクセルで打ち直すなどして作成する．
例としては下記のようになる．

| File id | 成否 |
|:-------:|:----:|
|0|TRUE|
|1|FALSE|
|2|FALSE|
|3|FALSE|
|4|TRUE|
|5|TRUE|
|6|TRUE|

このうち，４行だけ変更して，**validation**用のデータとして指定する．
具体的には，先程の`status.csv`の２列目を`TRUE`から`VALIDATION`にする．
４行だけ変更する意味は，４つの教師データをvalidation dataとして使用するという意味になる.

学習を行うプログラム内で，これらの値を使用して，教師データの処理を行う．

## 教師データの内容
４５種類のデータから構成されている

|Column index|Name|
|:--:|:----------:|
|0|dirt(x)|
|1|dirt(y)|
|2|dirt(z)|
|3|dirt(r)|
|4|dirt(p)|
|5|dirt(w)|
|6|goal(x)|
|7|goal(y)|
|8|goal(z)|
|9|goal(r)|
|10|goal(p)|
|11|goal(w)|
|12|broom(x)|
|13|broom(y)|
|14|broom(z)|
|15|broom(r)|
|16|broom(p)|
|17|broom(w)|
|18|joint state(s)|
|19|joint state(l)|
|20|joint state(e)|
|21|joint state(u)|
|22|joint state(r)|
|23|joint state(b)|
|24|joint state(t)|
|25|joint command(s)|
|26|joint command(l)|
|27|joint command(e)|
|28|joint command(u)|
|29|joint command(r)|
|30|joint command(b)|
|31|joint command(t)|
|32|pose state(x)|
|33|pose state(y)|
|34|pose state(z)|
|35|pose state(r)|
|36|pose state(p)|
|37|pose state(w)|
|38|cmd_vel(x)|
|39|cmd_vel(y)|
|40|cmd_vel(z)|
|41|cmd_vel(r)|
|42|cmd_vel(p)|
|43|cmd_vel(w)|
|44|leptorino(x)|
