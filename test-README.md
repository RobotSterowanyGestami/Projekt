# Używanie
blinker w cpp:
```g++ -Wall -pthread -o blink blinker.cpp -lpigpio -lrt``` 
```sudo ./blink``` 

### Koniecznie `sudo`, bo *pigpio* nie działa w cpp bez tego, problem z plikiem /dev/mem (https://github.com/fivdi/pigpio/issues/2)

blinker w py:
```python3 blinker.py```
