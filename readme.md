~ General Information

- Windows 11 OS
- Untiy Engine Version 2021.03.17f
- Anaconda3 latest version
- Python version 3.6.x
- ML Agents Release 6

# --------------------------------------------------------------------------------------------------

~ Setting Unity Packages

* Window -> Package Manager
ML Agents version 1.3.0: 
Press the plus buttom on the left upper corner and choose "add package from git url", 
then paste the url "git+https://github.com/Unity-Technologies/ml-agents.git?path=com.unity.ml-agents#release_6".

Universal RP latest version:
Search it by name inside the unity registery and you should find it.

AGX latest version:
  1. Download the AGX package from "https://us.download.algoryx.se/AGXUnity/".
  2. Import the package to unity (Assets -> Import Package -> Custom Package, and choose the AGX package).
  3. Insert your AGX lisence inside the lisences manager (AGXUnity -> Lisence -> Lisence Manager)

# --------------------------------------------------------------------------------------------------
~ Setting Unity Height Map

Using RP Universal package:
1. Create custom pipline Asset 
(right click in project assets tab, Create -> Rendering -> URP Asset with Universal Renderer)
2. Use a custom pipline Asset 
(Edit -> Project Settings -> Graphics).
3. Create a shader graph 
(right click on assets, Create -> Shader Graph -> URP -> Lit Shader Graph).
4. Double click on the new ShaderGraph asset, and create new vertex color 
(press space and enter "Vertex Color").
5. Drag the new vertex color to base color opetion under Fragment 
(each one of them has a little hole, drag hole to a hole).
6. Save your asset shader asset inside the ShaderGraph window 
(the buttom appears at the left upper corner).
7. Create a new terrain material, and inside the material inspector, change The default shader to the new shader graph 
(right click on assets, Create -> Material).
8. Create a new Height Map object (empty object with custom compoments), with: 
	- Mesh Renderer (make sure you inserted the privously created material inside the inspector).
	- Mesh Filter. 
	- Height Map Script(drage the desired terrain's terrain compoment inside the script).
9. Customise your Gradient so it would have unique colors in each height.
10. Make sure to choose appropriate offset.

# --------------------------------------------------------------------------------------------------

~ Setting ML-Agents Python API

* Python Packages
- ml_agents and ml_agents_envs version 0.19.0
- numpy version 1.18.5
- tensorflow version 2.6.2
- cudatoolkit version 11.2 

* Notes
- The cudatoolkit pack has to be downloaded as a conda package!!
- cudatoolkit version is overlapping tensorflow version 
(for instance, tensorflow version 2.6.2 would function properly with cudatoolkit version 11.2).

Python API Setup:
1. Install manually mlagents release 6 from 
"https://github.com/Unity-Technologies/ml-agents/blob/release_6_docs/docs/Installation.md".
* Inside the Command Prompt
2. Creating dedicated conda enviroment (inside The cmd type "conda create --name 'your-venv-name' python=3.6").
3. Activate conda new enviroment (conda activate "your-venv-name" or using conda prompt).
4. Change your directory to ml agents repo (cd C:\path\to\ml_agents_repo).
5. use pip to install ml_agents ,ml_agents_envs and numpy
(pip install -e ./ml-agents && pip install -e ./ml-agents-envs && pip install numpy==1.18.5).
6. Install cuda tool kit "conda install -c conda-forge cudatoolkit=11.2 cudnn=8.1.0".

# --------------------------------------------------------------------------------------------------

~ Possible Issues And Solutions

Audio Compoment Console Spam Soulotion:
1. Enter the WLO prefab (Assets\AGX\AGXUnity_WheelLoaderML\AGXUnity_WheelLoaderML_Assets)
2. Search for the CamFollow object inside the prefab (the search bar in the upper left corner), 
and delete the Audio compomnet.

Missing textures:
1. Enter the prefab data folder 
(Assets\AGX\AGXUnity_WheelLoaderML\AGXUnity_WheelLoaderML_Assets\AGXUnity_WheelLoaderML_Assets).
https://forum.unity.com/threads/cant-upgrade-materials-textures.842431/