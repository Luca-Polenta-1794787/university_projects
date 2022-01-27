# Mobile Applications & Cloud Computing

## Info
In this project, an android app was developed in Kotlin/Java through the use of Android Studio. The game is available in both light theme and dark theme. The features it had to have are as follows and were achieved in the following way:

- Use of graphics: both a "static" graphic that is interactively updated by the user, and a "dynamic" graphic where a series of containers are filled based on what is downloaded from a server and everything is placed in list containers was developed;
- Multi-users: possibility of access via firebase authentication or via Facebook web-api. Both then converge into a user page where user information and feature saves are collected in the app;
- Use of cloud services: use of a cloud service created by us on pythonanywhere for downloading and managing items in the shop and in the cart;
- Use of sensors: use of accellerometer, magnetometer and light sensor in the game; 
- Concurrency: Use of coroutines for downloading items in the shop and use of concurrency by extending the Thread class to update the graphics of the game in a separate thread from the main UI thread.

Both the game and the server will be available in the repo. Tha latter is managed through Restful and SQLite.

## Trailers

Login - Sign Up:
<p align="left">
<img src="sample_gif/example_login_signup.gif" height="30%"/>
</p>

Quiz:
<p align="left">
<img src="sample_gif/example_quiz.gif" height="30%"/>
</p>

Shop:
<p align="left">
<img src="sample_gif/example_shop.gif" height="30%"/>
</p>

Game:
<p align="left">
<img src="sample_gif/example_game.gif" height="30%"/>
</p>

## Group members
- Matteo Marini
- Luca Polenta
- Gianfranco Romani
