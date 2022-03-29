# Używanie
blinker w cpp:
```g++ -Wall -pthread -o blink blinker.cpp -lpigpio -lrt```
```sudo ./blink```

### Koniecznie `sudo`, bo *pigpio* nie działa w cpp bez tego, problem z plikiem>

blinker w py:
```python3 blinker.py```

