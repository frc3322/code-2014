@echo off
netsh interface ip set address name="Local Area Connection" static 10.33.22.6 255.0.0.0 10.33.22.4 1
pause
:: if this does not work, try changing the address name
