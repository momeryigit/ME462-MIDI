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
13. https://www.robolinkmarket.com/xl4016-300w-10a-dc-dc-voltaj-dusurucu-regulator-karti? elimizdeki 2 tane bundan var.
14. 

## MEETING NO: 6   
__DATE: 20.03.2024   
TIME: 17:35  
TOPIC:?__
 
__ATTENDEES: Ömer, Seçkin, Erdem, Sarp, Buğra hoca, Omar, İsmail Hoca__   
__Writer: Ömer__

1. Stepper motor drivers are getting hot. May need heat sinks. 2 drivers are burnt.
2. Dampers are good. Front and back axes should be balanced. Apparatus' length may change. We can extend the hinge.
3. Have some stuff around that resemble carpets, obstacles.
4. We'll need a makita charger.
5. We may need to build a localization stuff to see our robot's position. This localization stuff can be built with aruco markers on top of our robot and camera. up. Camera can tell our robot's position. Another possibility is the opposite. That's why Raspberry Pi is desired. One camera looks up to the ceiling which is filled with aruco markers and the other camera looks front.
6. Localization is extra.
7. Our robot should have bumpers, ultrasonic sensors ( not a big must ), accelerometer. And there is raspberry pi.
8. We have a laser scanner (SLAM side is opened).
9. We should be able to place this laser scanner into our robot (a meaningful place) and let it publish to a topic.
10. This laser scanner does not see glass.
11. Bumpers should not be metal. Touching part should not be metal.
12. There are some linux st keys for the scanner. This scanner is almost ros-ready.
13. It can become a 3D laser scanner (HOW COOL IS THAT? xD).
14. For hot swappable part: We may have 3rd battery. ALternatively, we need to come up with some capacitors, which is our buffer.
15. We should have a swap button. When pressed, computers should minimize its processes(image processors, laser data, etc). These processes should be pausable. When we are swapping the batteries, we can measure the time it takes and perform the calculations.
16. Pressing button idea can change. For example, it might be a switch.
17. We can do it with regular batteries instead of using capacitors.
18. We can put a tiny powerbank. We only want Pi to run forever.
19. We can purchase a UPS(Uninterruptable Power Supply) for Raspberry Pi.
20. Have the raspberry pie outside and let it get charged during the day(solar panels). Make some research about solar panels.
21. If there is a solution, we can go for it.
22. Also, there are some powerbank charges and discharges at the smae time.
23. Send .stl's to Buğra hoca and he may print them. Or get a TPU from him.
24. We may use WiFi instead of serial. Pi may not be next to ESP32.
25. Someone may put their laptop on top of our robot and do not even use raspberry pi. Wireless provides flexibility.
26. Wireless should be tried. Robot should support both (wireless and wired).
27. Publisher should be all around the world. Any PC on this network should be able to publish.
28. Generally, Ubuntu is put into pi. We can also run boot raspberry pi 4.
29. First boot will be on SD card (not clear).
30. Buğra hoca will give us an SD card.
31. We should SSH into pi (developer mode). When pi boots up, it should be ROS controllable.
32. Laser is connected to pi.
33. We can open 2 ports for downloading and uploading in MPython.
34. We may use Raspberry Pico or ESP32.
35. ESP's have pwm pins which do not put any CPU work. This may be true for pico as well.
36. Python may not be optimal for threading.
37. We might use a timer for publishing data. We can determine time periods.
38. Whenever a bumper is pressed, publish it(when there is a change). Battery may be updated once in 10 secs. These will be designed. Ultrasonic can be 10 Hz.
39. We should put some effort on localization. Also, the dead reckoning.
40. We can have some electric tapes and build grids. We can test our dead reckoning. We can figure out how much slippage we have. For example, we say go for 20 cms and test our robot.
41. Wide angle camera (more than 140 degrees). Try to capture images with it.
42. Major concern is good integrity. Things should be nicely plugged etc. Basic usage should be easy.
43. For future users, prepare a tutorial for each case. (Raspberry pi and PCs or only PCs). ROS2 packages should run on anywhere (PC, raspberry pi). Our tutorial should teach how to build network (wired? wireless?).
44. We must provide flexbility (pi pc robot, pc only robot, or anything that comes to mind).
45. Roll of TPU, Makita charger will be provided by Buğra hoca.
