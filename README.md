# 10 DoF Vehicle Dynamics Simulator

This project contains the code of the 10 degrees of freedom vehicle simulator that I developed during my PhD Thesis at Mines ParisTech, and used in many of my publications.

For full explanation of the model, please refer to the chapter 2 of my [PhD thesis](http://www.theses.fr/2018PSLEM025)

Examples of video obtained coupling this simulator to the rendering environment PreScan developed by TNO. In the first video, you can see the vehicle is loosing control due to slip.

<iframe width="560" height="315" src="https://www.youtube.com/embed/BRpmdIxTz-0?si=4PJj1-yUX20Vbwf_" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

- [High speed trajectory planning](https://www.youtube.com/watch?v=BRpmdIxTz-0&feature=youtu.be)

- [Rain Condition (road coefficient $\mu$ set to 0.7)](https://www.youtube.com/watch?v=6LFNhpcmssY)

- [Snow Condition (road coefficient $\mu$ set to 0.2)](https://www.youtube.com/watch?v=qUT5sFY_RE4)

- [Deep Learning for coupled longitudinal and lateral control](https://www.youtube.com/watch?v=yyWy1uavlXs)

Known Limitations:

- missing Ackermann steering wheel (both front wheels have the same angle)

Credits:
Please quote my PhD if using my simulator for publication or whatever other use you may have of it.

## Installation

Install dependencies

```bash
sudo apt install libeigen3-dev
sudo apt-get install libboost-all-dev
```

Build project

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

## License

[GNU GENERAL PUBLIC LICENSE](https://www.gnu.org/licenses/gpl-3.0.html)