# 품평회 멘토링
 
# AVR을 이용한 PCM 오르골
> PCM Orgel Using AVR

## 목차
 1. TICK타이머 구현
 2. GPIO 입력부 구현
 3. 외부 인터럽트 이용한 입력부 구현
 4. PCM 데이터를 통해 스피커 출력부 구현
 5. eeprom을 이용하여 녹음기능 구현

### TICK타이머 구현

타이밍을 중요하게 고려해주어야 하는 시스템의 경우 tick 타이머가 큰 도움이 된다. 

여기서 tick 타이머란, 시스템에서 제공하는 어떤 시간(?)이라고 생각하면 될까 싶다... 이 tick 타이머를 통해 시간을 만들어낼 수 있다. 가령 '전원을 인가하고 나서 1분 후', '지금으로 부터 10초가 경과한 순간', '매 순간 1초마다 어떤 명령을 수행해라' 등등... 이런 것들이 감지가 tick타이머가 있으면 가능하게 되는 것이다.  

대부분 tick 타이머는 1ms(1kHz)짜리 타이머를 만들어주는 것이 일반적인 것 같다. 각 개발자들 마다 의도하는 바가 다를 수 있기 때문에 항상 1ms라고 단정지을 수는 없지만, 여튼 대부분이 1ms로 만들어놓고 사용한다. 

사용하는 예시는 다음과 같다

 - tick 타이머인 'ticks'라는 변수는 1ms마다 1씩 증가하는 변수이다.
 ```C
 
 while(1){
 
   //ticks가 계속 증가되면서 1000ms == 1초가 되면 do_action()이라는 함수가 호출된다.
 
   if(ticks%1000=0) do_action();
 
 }
 ```
 
 
 - 그렇다면 ticks는 대체 어디서 증가하는 변수인걸까?
 
그 정답은 타이머카운터의 인터럽트를 이용하는 것에 있다. 

좀 더 자세히 설명해보면, 1ms마다 타이머 카운터의 인터럽트(비교매치던 오버플로던)가 발생하도록 하여 인터럽트 서비스 루틴(이하, ISR)에서 ticks 변수를 증가시키도록 한다.

tick 타이머를 구현하는 것에 있어서는 직접 데이터시트를 확인하여 구현하면 된다.

확인해야 하는 [레지스터](https://blog.naver.com/uu5626/221439659186) 갯수도 몇개 없고 약간의 산수만 해주면 손 쉽게 구현이 된다.







### GPIO 입력부 구현

24개 키 입력을 동시에 받는 일은 쉽지 않다. (버튼, 스위치 같은 하드웨어상의 부품들은 풀업/풀다운을 통해 로직상태를 확실히 보장도 해주어야 하며, 뿐만 아니라 채터링 이슈까지도 신경 써줘야 한다.)
그렇기 때문에 가장 일반적인 생각으로는 버튼 하나에 풀업 또는 풀다운 저항이 구성되어야 하며, 채터링을 방지하고자 바이패스 커패시터 또한 구성되어야 한다. 

그런데 24개 버튼 모두에 저항과 커패시터를 달아준다는 일은 여간 쉬운 일이 아니다. 다만 이런 과정을 거쳐준 다면, 버튼 입력에 노이즈가 끼는 현상이 줄어들며, LOW/HIGH신호가 정상적으로 입력되는 것이 보장된다. 즉, MCU에 연결된 버튼의 경우 이상한 데이터가 들어오지 않고 개발자가 원하는 데이터가 정상적으로 들어오기 때문에 코드 작성하는데 고려해줘야 되는 부분이 덜해진다.

앞서 말했지만, 버튼 24개에 각각 저항과 커패시터를 달아주는 것은 번거로우며 회로 사이즈가 커지기도 하며, 양산을 한다면 단가가 상승하는 원인이 될 수 있다. 그렇다면, 이에 대한 해결책은 저항과 커패시터를 달지 않고 풀업/다운을 처리해줄 수 있고, 채터링 또한 잡아줄 수 있는 어떤 시스템을 소프트웨어를 통해 구현해주는 것이다. 다만, 코드가 조금 복잡해지는 것은 피해갈 수 없다.

여기서 Trade Off가 발생할 수 있다. __[코드의 복잡도 <--> 회로의 복잡도]__  

현재 만들고 있는 작품의 여건 상, 소프트웨어 적으로 처리해주는 방법을 채택해주기로 한다.

__소프트웨어로 GPIO 입력 처리 하는 과정__

- GPIO 입력은 총 24개이며, A포트, C포트, F포트를 전부 사용하여 입력을 받기로 한다. 

- 풀업의 경우 GPIO 레지스터 설정을 통해 내부 풀업을 사용한다.

- 각 포트의 상태를 검사하는 행위를 scan이라고 정의하도록 한다. 

- key scan은 main loop에서 매번 하는 것이 아니라, 10ms마다 한 번씩 스캔하는 것으로 처리한다.

- key scan 시, A,C,F 포트 중 어느 하나라도 버튼 입력이 인식이 되면 엣지 검출용 'edge_detect' 플래그가 set되도록 하여, 해당 루틴에 단 1회만 들어가도록 처리해준다.

- 버튼을 모두 Release 해주었을 경우에는 엣지 검출용 플래그가 다시 clear되도록 코드를 구성해주면 된다.

- 이렇게 해줄 경우 장점은 아래와 같다.
  
  - 10ms마다 검사를 하며, 어느 순간 입력이 detect되면 엣지 동작을 수행할 뿐더러, 10ms 사이 동안엔 추가적인 scan행위를 하지 않는다.
  
  - 즉, 엣지 검출이 가능함과 동시에 채터링까지 잡아주는 효과를 얻어낼 수 있다.
  



### 문제 발생 및 고려 사항 등등

 - 인터럽트 발생 빈도가 너무 빠르다면 다른 시스템에 영향을 준다. 예를 들어, TIM0 비교매치 인터럽트 주기가 32kHz인데, 이것 말고도 다른 인터럽트(A인터럽트라고 가정)도 사용을 한다고 할 때,
 A 인터럽트는 10ms마다 발생한다고 칠 때, TIM0 비교매치 인터럽트가 A 인터럽트가 발생하는 순간순간마다 뺏어가는 상황들이 발생하여 A 인터럽트 동작에 부정적인 영향을 끼친다. 
 
 
 - 뿐만 아니라, main루프에도 좋지 않은 영향을 미친다. main 루프에 딜레이가 들어가는 상황을 만드는 것은 가급적 피하는 것이 좋다. 만일 인터럽트가 너무 자주 발생한다면 이는 마치 main루프에 딜레이를 거는것과 비슷한 결과를 낳는다. 

 - 해당 문제에 대한 결론은 다음과 같다. 
   - 시스템 클럭이 16MHz인 것을 고려할 때, 32KHz속도로 발생하는 인터럽트를 만들어주는 것은 애초에 설계부터 잘못되었다.(16MHz로 동작하는 시스템에서 32kHz마다 발생하는 인터럽트는 시스템 오동작에 원인이 되기 쉽다.)
  
   - 해당 시스템에서 32KHz로 만들어줄 수 밖에 없는 상황이라면, ISR 루틴에 코드를 최대한 간결하게 작성해주어야 한다. 
  
   - 1ms짜리 tick Timer는 main문에서 만들어주도록 한다. 
  
   - 다음 부턴 비슷한 상황이 생긴다면, 인터럽트 발생 속도를 낮추던지, 시스템 클럭이 월등히 높은 MCU를 선정하는 판단이 필요하다. 
  
  
[^1]: TIMSK
  
  
  
