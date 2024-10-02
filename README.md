# CrEGOpt: Cross Entropy Gait Optimization For Legged Systems

Official code implementation of the paper entitled *Gait Optimization for Legged Systems Through Mixed Distribution Cross-Entropy Optimization* by Ioannis Tsikelis and Konstantinos Chatzilygeroudis.

See more at [https://nosalro.github.io/cregopt](https://nosalro.github.io/cregopt).

<p align="center">
<img src="https://github.com/NOSALRO/cregopt/blob/master/docs/static/images/concept_cregoptv1.png" alt="concept_figure" width="50%"/>
</p>

## Maintainers

- Ioannis Tsikelis (University of Patras) - tsikelis.i@protonmail.com
- Konstantinos Chatzilygeroudis (University of Patras) - costashatz@upatras.gr

## Citing CrEGOpt

If you use this code in a scientific publication, please use the following citation:

```bibtex
@inproceedings{tsikelis2024gait,
      title={Gait Optimization for Legged Systems Through Mixed Distribution Cross-Entropy Optimization},
      author={Tsikelis, Ioannis and Chatzilygeroudis, Konstantinos},
      booktitle={IEEE-RAS International Conference on Humanoid Robots (Humanoids)},
      year={2024}
    }
```

## Replicating the paper results

Clone this repository.

```shell
git clone git@github.com:NOSALRO/cregopt.git
```

The required libraries are added as submodules.

```shell
cd cregopt/.ci/
git submodule init
git submodule update
```

\*This code uses the HSL_MA97 parallel solver package for IPOPT, which is licensed \(more info [here](https://licences.stfc.ac.uk/product/coin-hsl)\). If you have the .zip file, you should extract its contents in the .ci/coinshl folder.

Build the docker image.

```shell
docker build -t ipopt .
```

Start the docker container.

```shell
cd .. # cd to the project root directory
docker run --rm -it --net=host -e DISPLAY -v ${HOME}/.Xauthority:/home/robot/.Xauthority -v "$(pwd)":/home/robot/code --entrypoint /bin/bash ipopt
```

In the docker container build the experiment executables and run the bash script that generates and stores the resutls.

```shell
cd code/
mkdir build && cd build
cmake ..
make -j
cd ..# cd to project's root directory
cd tools/

./run_experiment_1 # Run 1st paper experiment.
./run_experiment_2 # Run 2nd paper experiment.
./run_experiment_3 # Run 3rd paper experiment.
```

The experiment results will be automatically exported in the [project root directory]/tools/results in csv form.

## Acknowledgments

This work was supported by the [Hellenic Foundation for Research and Innovation](https://www.elidek.gr/en/homepage/) (H.F.R.I.) under the "3rd Call for H.F.R.I. Research Projects to support Post-Doctoral Researchers" (Project Acronym: NOSALRO, Project Number: 7541).

<p align="center">
<img src="https://www.elidek.gr/wp-content/themes/elidek/images/elidek_logo_en.png" alt="logo_elidek"/>
<p/>

This work was conducted within the [Laboratory of Automation and Robotics](https://lar.ece.upatras.gr/) (LAR), Department of Electrical and Computer Engineering, and the [Computational Intelligence Lab](http://cilab.math.upatras.gr/) (CILab), Department of Mathematics at the University of Patras, Greece.

<p align="center">
<img src="http://lar.ece.upatras.gr/wp-content/uploads/sites/147/2022/10/lar_profile_alpha.png" alt="logo_lar" width="20%"/><br/>
<img src="https://nosalro.github.io/images/logo_cilab.jpg" alt="logo_cilab" width="50%"/>
<img src="https://www.upatras.gr/wp-content/uploads/up_2017_logo_en.png" alt="logo_upatras" width="50%"/>
</p>
