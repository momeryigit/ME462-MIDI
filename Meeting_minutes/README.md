# METU ME ME462 MIDI ROBOT PROJECT MEETING MINUTES   
## MEETING NO: 1   
__DATE: 28.02.2024   
TIME: 16:00   
TOPIC: FIRST MEETING__
 
__ATTENDEES: Buğra Hoca, İsmail Hoca, Ömer, Omar, Seçkin, Erdem__   
__Writer: Erdem__

1. Gİthub oluşturulacak
1. Makita matkap şarjıyla çalışacak 1-2 üç tanesiyle belki
1. Şarj arayüzü olacak, makita şarj aletinden gidecek belki kabloyla birşleşecek şarjı çıkarmadan şarj edecek
1. Pil ile robot arasında bağlantı yapmak için mekanik parça lazım bu halihazırda tasarlanmış, onun tasarımını iste
1. Batarya voltajını ADC converter ile ölçerek ne kadar pil kaldı onu anla.
1. Bİr planner tasarla ne kadar şarj kaldı onu anla ve önündeki taskı tamamlayabilir mi onu kontrol etsin ona göre araya şarj etme operasyonunu araya soksun
1. Bİr shut down prosesi tanımla, örneğin butona bağla ve butona bağlarsan ne zaman kapanması gerektiğii anlar ve kendini proper birr şekilde kapatır.
1. UPS gibi bilgiayar kapanmadan nasıl batarya değiştirirsin onu düşün ve çözüm önerisi sun
1. farklı kullanım senaryolarıyla gelecek herkes haftaya, herkesinki farklı olacak. her case için ne özellikler olması lazım onları açıkla, soft or hard recoommandations şeklinde. kesin olacak ve olmayacaklar şeklinde.
1. Gantt chart hazırla, dönem boyunca kimse tek başına çalışmayacak takımlar sürekli değişecek
1. Hot swappable batteries
1. asıl fikir üniversitelerin liselerin vs. alıp kullanabilecğei açık bir platform yapmak.
1. fablab?
1. 3d printing, fdm only, plexiglass or wood but not metal, NO CNC maybe drill press, sigma profiles, 
1. Sensors like RGB cameras depth cameras, velodyne 3d scanner Raspberry pi inside, someone can put a laptop on it maybe, 
1. 20 kgs of payload
1. bir insan üzerine çıkıp zıplarsa kırılmayacak ama kırılırsa da belirli bazı parçalar kırılacak ve onlar hızlıca değiştirilebilecek.
1. genel robot etrafında güç ve haberleşme portları olacak ve böylece çeşitli şekilde farklı sensör vs takılabilecek. 
1. bir başka seçenek de esp32 veya pi pico gibi bir şeye adapte olabilecek bir haberleşme arayüzü tanımlmaka ve müşteri kendisi mikroişlemciyi alacak sistemini kuracak mikro ros çalıştıracak ve entegre olacak.
1. Status light olmalı barebone robotta 
1. ROS2de parameter topic farkı nedr bir bak.
1. RAspberry pi 5 olabilir hoca bulabilirse, bir kamera yukarıya bakacak bir kamera ileriye bakacak. 
1. Cuma vs. checklist yapılacak ME407deki design criteria ve project requriements vs.  sonra haftaya çarşambaya da gantt chart ve planlama yapılacak. 
1. Airless tire tasarımı yapılacak TPU materialdan
1. NEMA 17 stepper motors


## MEETING NO: 2 with Bugra Hoca
__DATE: 06.03.2024   
TIME: 16:00   
TOPIC: ?__
 
__ATTENDEES: Buğra Hoca, İsmail Hoca, Ömer, Omar, Seçkin, Erdem, Sarp__   
__Writer: Sarp__

1. Slam with color camera, slam with depth sensor (high level, be able to provide these data)
2. This is not a robotics project, aim is to provide options, data, documentation and ease to the user.
3. Find a guinue pig.
4. Implement ROS, provide documentation about implemention of ROS.
5. Rwiz is considered High Level, Try to apply all low level implementations, such that end user can apply them in high level apllications.
6. Aruco markers
7. Provide different ways of controlling motors.
8. See if you can integrate phone sensonrs with ROS. Hoca will love this.
9. For use cases, consider co-operation between other devices. Use cases are for us to implement its low level requirements and make the robot much more flexible.
10. Bumpers (Barebones)
11. Should not fall down from stairs (Ultrasonic sensors looking below.)
12. Consider current sensors.
13. Remind Bugra Hoca about ESP32, cnc shield, and stepper drivers
14. Make things move around as soon as possible.
15. Stepper disabling when idle, while making sure not to roll down a hill


## MEETING NO: 4   
__DATE: 11.03.2024   
TIME: 18:30   
TOPIC:?__
 
__ATTENDEES: Ömer, Seçkin, Erdem, Sarp__   
__Writer: Erdem__

1. Micro ros and ROS2 have been discussed and researched.
2. Makita charger theory have been discussed and made research.
3. TPU prints have been examined.
4. New 3d prints have printed.
5. Seamless charging dock integration have been discussed.
6. Not existing parts have printed.


## MEETING NO: 5   
__DATE: 13.03.2024   
TIME: 16:15  
TOPIC:?__
 
__ATTENDEES: Ömer, Seçkin, Erdem, Sarp, Buğra hoca, Omar, İsmail Hoca__   
__Writer: Erdem__

1. Flex tekerler hocaya gösterildi.
2. Sarp teker jantlarını romerde basmış, tekerler hazır halde sadece gövdeye monte edilecek.
3. Gövdenin daha çok metal olması ve pleksiler içermesi konuşuldu ve hoca onayladı ama ortadaki gereksiz metal supportları kaldırabilirsiniz dedi.
4. ESP8266 microros desteklemiyormuş.
5. robot açılınca etraftaki kameralara bağlanmaya çalışabilir, eğer yoksa kendi sensörleriyle yola devam edebilir.
6. Hoca bizden kamera infosunu kullanmamızı beklemiyor, high functionlaity vs. beklemiyor, sadece bağlanmamızı bağlanabildiğimizi göstermemizi bekliyor.
7. micro ros için freeRTOS mu yoksa arduino mu yapmalıyız konuşuldu, hoca pure python olsun dedi.
8. mikroişlemcide çalışacak olan şeyler: ledler, bumper sensörler, belki küçük bir ekran onun dışında büyük sensörler ana bilgisayarda rosta dönecek.
9. Micro rosa girmeden esp32 esp8266 veya pi pico ile normal bildiğimiz kod yazıp port açıp rosla haberleşme yapmak da okey ve daha iyi de olabilir dedi hoca.
10. Pythonla yazılacağı için c olmayacağı için readability daha iyi olacak.
11. Self charging özellik olmasa da olabilir şart değil hatta şimdilik bunu erteledik sadece hot swappable batteries olarak yapılacak.
12. Hot swappable battery konsepti araştırılacak ve orada neler yapılabilir bakılacak, parallel connection olması iyi mi kötü mü bir sorun olur mu araştırılacak haftaya.
13. 
