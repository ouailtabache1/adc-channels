ESP-IDF template app
====================

# Create a multichannel A/D application 
It is a single channel A/D application with moving hysteresis & moving average & calibration. 
Convert it to multiple channel app. You can find some suggestions in the code with 
```
//!!!
```

I suggest create a new branch from master to implement yourr task and finally merge it to master when it works well:

```
git checkout -b multi_channel
git commit -am "" ...
git commit -am "" ...
git commit -am "" ...
git commit -am "" ...
git checkout master 
git merge multi_channel
```


good luck
Zamek
