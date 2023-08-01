# Step-MotorDriver-with-STM32
Kartezyen sistemde step motor kontrol sağlayan çalışmam

Çalışma içerisinde Step motorların sürülmesi için gerekli olan pin çıkışları ve işlevlerini yazdım aynı zamanda step motor ile sürücü bağlantısını burada sizlerle paylaşacağım.

PC10 -> DIR+
PC12 -> EN+
PC11 -> PUL+

PA0 -> POT
PA2 -> POT2
PC9 -> Button Yön
PC8 -> Button
PB8 -> Limit Switch

DIR pin ile yön kontrolü - Step Pin ile hareket sağlanmaktadır - EN pin ise EN = 0 Açık EN = 1 kapalı hale getirmektedir.

![2 gün](https://github.com/MHI-Embedded-Dev/Step-MotorDriver-with-STM32/assets/132624287/89590b76-287d-49bc-970a-2e300c9bbc15)

Bağlantı şeması bu şekildedir.


Merhabalar bu çalışmayı gerçekşeltirme imkanı sunan ByteSpark firmasına ve ekibine teşekkürler...
