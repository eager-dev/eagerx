# Changelog

<!--next-version-placeholder-->

## v0.1.32 (2022-09-13)
### Fix
* Deepcopy engine, list `EngineNodes` without outputs as dependency ([#215](https://github.com/eager-dev/eagerx/issues/215)) ([`ae6a23a`](https://github.com/eager-dev/eagerx/commit/ae6a23a8e237a9a33906fea760c5805b3830b982))

## v0.1.31 (2022-09-12)
### Fix
* Substitute `Object` graphs into the agnostic `Graph`. ([#210](https://github.com/eager-dev/eagerx/issues/210)) ([`cb61dfa`](https://github.com/eager-dev/eagerx/commit/cb61dfa212ad5ccec45152677666df952132b740))

### Documentation
* Remove clutter from readme ([`378b599`](https://github.com/eager-dev/eagerx/commit/378b599651655d9cce13bf909b0337e8616b8561))
* Increase size gui image ([`f630e80`](https://github.com/eager-dev/eagerx/commit/f630e80441b93ee4881681113b41d3a2d3475477))
* Make readme less wordy ([`4ea2655`](https://github.com/eager-dev/eagerx/commit/4ea26550dcccac4aec79fad0845c49d3b1d2fcf1))
* Fix links ([`8e60dc7`](https://github.com/eager-dev/eagerx/commit/8e60dc7adeddb858a19e4410849c4580dab5034d))
* Fix broken link ([`6781f86`](https://github.com/eager-dev/eagerx/commit/6781f86482b55dad8d0bad978b8221f5090622f7))
* Readme from rst to md and added gifs, code example and available engines ([`8a44cdb`](https://github.com/eager-dev/eagerx/commit/8a44cdb519a2f172ba7572a0def01963edf45a1d))

## v0.1.30 (2022-08-02)
### Fix
* Filename argument now passed to gui render function ([`9344240`](https://github.com/eager-dev/eagerx/commit/934424018b59122094c49c3fcb2c83b43c9b2dc5))

## v0.1.29 (2022-08-02)
### Fix
* Refactor shape to resolution in gui and add filename arg ([`a9943e4`](https://github.com/eager-dev/eagerx/commit/a9943e493a26fb271390614a8866e5f372219a14))

## v0.1.28 (2022-08-01)
### Fix
* Add interactive and shape arguments to gui methods for headless rendering of the gui ([`11e91ea`](https://github.com/eager-dev/eagerx/commit/11e91ea212577e48d9f177c20172caca182ccfc7))

## v0.1.27 (2022-07-24)
### Fix
* Info bug for objects  ([`92db335`](https://github.com/eager-dev/eagerx/commit/92db335de475e6a1a89de6eeeff318261a0b659e))
* Load and reload entities when loading graph from yaml file ([`07a11a4`](https://github.com/eager-dev/eagerx/commit/07a11a49181fa06634e12fe95c2693b4ab639c6e))
* Bgr to rgb encoding in colab renderer ([`692b56f`](https://github.com/eager-dev/eagerx/commit/692b56f687811a5a954566880332bdc580eb1db8))

### Documentation
* Add object tutorial ([`d57acd6`](https://github.com/eager-dev/eagerx/commit/d57acd63157c5933fc468e03c431ba1fdcd49904))

## v0.1.26 (2022-07-20)
### Fix
* Remove dependency on genpy ([`d909b52`](https://github.com/eager-dev/eagerx/commit/d909b52d7c1ef5265a7aca8a22110054240579e5))

## v0.1.25 (2022-07-20)
### Fix
* Add backend, numpy messages, remove converters ([#189](https://github.com/eager-dev/eagerx/issues/189)) ([`77a6b26`](https://github.com/eager-dev/eagerx/commit/77a6b26e6155e050914ad207176500a977fcffd6))

### Documentation
* Add introduction tutorials and remove developer guide and advanced tutorial ([`7ce68ea`](https://github.com/eager-dev/eagerx/commit/7ce68ea9099cce66f7d44fcea547cb564d99a931))
* Update conda env ([#178](https://github.com/eager-dev/eagerx/issues/178)) ([`a1ba574`](https://github.com/eager-dev/eagerx/commit/a1ba574e346c12d33473d13b2991a1bbe33ad60e))
* Add expected tutorials and stable release disclaimer ([`cb4c280`](https://github.com/eager-dev/eagerx/commit/cb4c28017261a77be6797132b8a7d588b1277e1e))

## v0.1.24 (2022-05-17)
### Fix
* Refactor `Env` to `BaseEnv`, add deprecation warnings ([`c632978`](https://github.com/eager-dev/eagerx/commit/c63297870afe587a3c10b6e1c4d063a9ffbb7b0a))

### Documentation
* Refactor `EagerxEnv` to `BaseEnv`. ([`4473354`](https://github.com/eager-dev/eagerx/commit/447335445cc9c1812a735084e80a66a276d162cd))

## v0.1.23 (2022-05-16)
### Fix
* Refactor bridge to engine ([`372e71c`](https://github.com/eager-dev/eagerx/commit/372e71ccd2782ffa80b9fb657b5e84b560cb17af))
* Update docs and remove initialize_spec ([`4bc3fdb`](https://github.com/eager-dev/eagerx/commit/4bc3fdbb89e24e9d0908b34dcb1e961bdcb4507f))
* Initialize spec inside `register.spec` wrapper. ([`e31f2fc`](https://github.com/eager-dev/eagerx/commit/e31f2fcf3f5513b83c1104d3bf47da60a76a4306))
* Get spec object from graph state ([`7b3c90f`](https://github.com/eager-dev/eagerx/commit/7b3c90f1ddca2807a55eaca7207f447dfc5d23e8))
* Ensure target and source are compatible ([`5a60c4d`](https://github.com/eager-dev/eagerx/commit/5a60c4d648a33d30e8d53b904687311ca7336a6f))

## v0.1.22 (2022-05-07)
### Fix
* Bug when checking window > 0 if skip=True & intial_obs!=None. ([`f45b33a`](https://github.com/eager-dev/eagerx/commit/f45b33ad69eaa37d664c1282ee36a56adf888da0))
* Avoid occasional opening of  empty window on shutdown. ([`5c0b657`](https://github.com/eager-dev/eagerx/commit/5c0b657e5def1e8f01120feb0da0e7d44bd32f1d))

### Documentation
* Add tutorial 5,6,7 ([`cf85123`](https://github.com/eager-dev/eagerx/commit/cf851237b8f8128cc81f5fcc2096855a2c4172e0))
* Fix broken links and ad poetry installation instructions ([`2d60d29`](https://github.com/eager-dev/eagerx/commit/2d60d29190095c6b45cd9c7f9c7cd19e367fb7b3))

## v0.1.21 (2022-05-05)
### Fix
* Put rendering in same thread. Set default to NEW_PROCESS. ([`b6d35d3`](https://github.com/eager-dev/eagerx/commit/b6d35d343881100e3c607e38cb0a857d5634dba8))

## v0.1.20 (2022-05-04)
### Fix
* Converter bug with bridge inputs ([`a5d9a8d`](https://github.com/eager-dev/eagerx/commit/a5d9a8d3e75900472b14787799784340c802873d))
* Create colab render window in first callback call.  ([`55d54ee`](https://github.com/eager-dev/eagerx/commit/55d54ee708601d2c6bbe90c19d30f80531232972))

### Documentation
* Rename ros to ros1 and add prerequisites to installation instructions ([`3753627`](https://github.com/eager-dev/eagerx/commit/3753627df904afb131ed42b6cc46effb43e0a92a))
* Add info on docker and installation from source ([`7872ce6`](https://github.com/eager-dev/eagerx/commit/7872ce6e51c76dca696f6c8733c4acd6850d59f9))
* Add docstring to `GymBridge.add_object`. ([`e1b225c`](https://github.com/eager-dev/eagerx/commit/e1b225c7e9de0db16176e219726a6ccd2156322f))
* Add colab tutorials ([`6c3202b`](https://github.com/eager-dev/eagerx/commit/6c3202b26eafa2468b3999fe3c88813c19909979))

## v0.1.19 (2022-05-03)
### Fix
* Allow bridge implementations to be functions ([`ac24db6`](https://github.com/eager-dev/eagerx/commit/ac24db6c38643070186b1cda47a5b46734a2dcd8))
* Render gym images on virtual display ([`f69b51a`](https://github.com/eager-dev/eagerx/commit/f69b51a970f0e965c06d9029813e1eb5d151b9a9))

### Documentation
* Update bibtex typo ([`0b10967`](https://github.com/eager-dev/eagerx/commit/0b10967b0aea06c3e5cab07e6b8f576fb2c385ab))
* Add std_srvs.srv to conf.py ([`b05e0af`](https://github.com/eager-dev/eagerx/commit/b05e0aff0e2e575dbc857859300c6d68c0cdd19d))

## v0.1.18 (2022-04-29)
### Fix
* Add colab render ([`f20d09d`](https://github.com/eager-dev/eagerx/commit/f20d09da01972732df5fd203f9105c8ac94d2ab5))

## v0.1.17 (2022-04-25)
### Fix
* Remove unnecessary SpaceConverter call. Closes #158. ([`b61be97`](https://github.com/eager-dev/eagerx/commit/b61be97c81dd014acb9b692752efd10e4b4beb34))
* Make mapping optional ([`11820dd`](https://github.com/eager-dev/eagerx/commit/11820ddff76e852fa188ff006c4fd616e65f2692))
* Refactor is_reactive to sync. Closes #149. ([`178255c`](https://github.com/eager-dev/eagerx/commit/178255c9aa16c8a274601f517887d694b14b0db9))
* Type check the output of callbacks ([`f29de1d`](https://github.com/eager-dev/eagerx/commit/f29de1dd00a37c17403e4d21aa9056e609719387))
* Only add EngineNode outputs as bridge input if `tick` is an input. ([`745631c`](https://github.com/eager-dev/eagerx/commit/745631c82043aab1bc4cae481b8577d70e75ee70))
* Remove inputs from bridge callback ([`024725f`](https://github.com/eager-dev/eagerx/commit/024725fa4f1df6ff93bc25eab4e9f029131656d7))
* Change default color of actuators and sensors ([`d6de36d`](https://github.com/eager-dev/eagerx/commit/d6de36d71c1bc8b24935524d77f6b4f8e0a40915))
* Select all registered components per default. ([`51f0deb`](https://github.com/eager-dev/eagerx/commit/51f0deb1ad8a8475a716460f08c3e4707fe80ccc))
* Downgrade already initialized roscore warning to info. ([`9bb6107`](https://github.com/eager-dev/eagerx/commit/9bb61079d3aad4a20af1cb82b358c1c6a1872c6b))
* Remove circular imports ([`7d600c7`](https://github.com/eager-dev/eagerx/commit/7d600c77e4645688e2830ac4b8601cb700c9424f))
* Allow setting the render node process. Per default in environment. ([`6a19fa0`](https://github.com/eager-dev/eagerx/commit/6a19fa042edee86ffd7bcae397b243ac093da36d))
* Imports in core nodes ([`79ed11a`](https://github.com/eager-dev/eagerx/commit/79ed11a5700abf07cbf06af82e86043173c59592))
* Remote shutdown if environmen with same name already exists. ([`5c38bce`](https://github.com/eager-dev/eagerx/commit/5c38bce321f1d680dad1beea8054d386ebc55409))
* Unregister render subs/pubs ([`5581be0`](https://github.com/eager-dev/eagerx/commit/5581be054eb654c5e01d8c352421cace15553455))
* Engine_state info ([`dc3181e`](https://github.com/eager-dev/eagerx/commit/dc3181e6e9c9d5e1c1b3ed7db36035baaf10a7a0))
* Allows specification of initial_obs if skip=True and window>0 ([`7628ec4`](https://github.com/eager-dev/eagerx/commit/7628ec451608085f04f50dca37ad6be3939fe96e))
* Allow keyword updating of specs ([`c933459`](https://github.com/eager-dev/eagerx/commit/c933459f0caf6e267218d05198efd65ae331fd30))
* Remove kwargs from Float32MultiArray ([`9f15cb9`](https://github.com/eager-dev/eagerx/commit/9f15cb92f51f28c558cee0884c810de4651d492b))
* Add ros to python path in colab ([`2cfa2d4`](https://github.com/eager-dev/eagerx/commit/2cfa2d41e76c0a3353eb6252a3fbac4b133a8e8a))

## v0.1.16 (2022-04-14)
### Fix
* Clean eagerx imports ([#154](https://github.com/eager-dev/eagerx/issues/154)) ([`999c449`](https://github.com/eager-dev/eagerx/commit/999c4496a9d4be6c1047a546798c5b07bde7e3d7))

## v0.1.15 (2022-04-14)
### Fix
* Reloading specs and add informative info. ([#153](https://github.com/eager-dev/eagerx/issues/153)) ([`0775715`](https://github.com/eager-dev/eagerx/commit/0775715af86929fc1f41309b8b35b14c18bdb8c4))

### Documentation
* Add missing sections (visualization, processor, converter, engine graph) + restructure + minor changes ([`fceaf90`](https://github.com/eager-dev/eagerx/commit/fceaf9001cd732aa80c2eec7a0858642414e4133))

## v0.1.14 (2022-04-07)
### Fix
* Update tests ([`7af6699`](https://github.com/eager-dev/eagerx/commit/7af66995542413e85e10caeb6e0f79e8ebff8beb))
* Bump gym to 0.21.0 and resolve flake8 dependency clash ([`ea1299c`](https://github.com/eager-dev/eagerx/commit/ea1299cad53e45c1b763efd290f7e88382321a0a))
* Add Object API to EngineGraph.gui() ([`351bfc1`](https://github.com/eager-dev/eagerx/commit/351bfc1f50d7c82937cc5b9caf0e712978388ab6))

### Documentation
* Add link to github repo in readthedocs ([`725e734`](https://github.com/eager-dev/eagerx/commit/725e734eb4bb7587015b80840cd5765637c600be))
* Add ros2 warning & known issue for anaconda with Qt ([`060b78c`](https://github.com/eager-dev/eagerx/commit/060b78ccc3b16bbc2a8bd1bb3c5304a91d3fa482))

## v0.1.13 (2022-04-06)
### Fix
* Bug with len(targets) > 0, refactor test_bridge to tests dir ([#144](https://github.com/eager-dev/eagerx/issues/144)) ([`8d1062e`](https://github.com/eager-dev/eagerx/commit/8d1062efa6e7d56447c512c3b3d49f55e51efacb))

### Documentation
* Pendulum tutorial ([#140](https://github.com/eager-dev/eagerx/issues/140)) ([`b1d668c`](https://github.com/eager-dev/eagerx/commit/b1d668cdea24250dabec9092c2913075f57f26db))

## v0.1.12 (2022-03-30)
### Fix
* Dummy commit to trigger release (attempt 2) ([`9adf59e`](https://github.com/eager-dev/eagerx/commit/9adf59e90215ee8fe19a4a574d6e54d250f0b2b2))
* Dummy commit to trigger release ([`232681e`](https://github.com/eager-dev/eagerx/commit/232681e26478504c20f224a9628e5f470e26ca6a))

## v0.1.11 (2022-03-29)
### Fix
* Access object configs in NP EngineNodes. Allow sens=0, act=0 ([#134](https://github.com/eager-dev/eagerx/issues/134)) ([`8f2f19e`](https://github.com/eager-dev/eagerx/commit/8f2f19e456645e4d23ea0c0a0dc4ddefef6e6790))

### Documentation
* Update logo ([`e5c6352`](https://github.com/eager-dev/eagerx/commit/e5c635261996370f7e29ef598952a6ddf78e2828))
* Add default values in spec, thicken lines in logo, update href ([`b3d7522`](https://github.com/eager-dev/eagerx/commit/b3d75224c81a67ff8e0e02ef095aa786bd500eb1))
* Fix cv_bridge import ([`9d13d58`](https://github.com/eager-dev/eagerx/commit/9d13d5880fc2c023b9d18d3a21678547e111ab6c))

## v0.1.10 (2022-03-18)
### Fix
* Docs api ([#126](https://github.com/eager-dev/eagerx/issues/126)) ([`efa505c`](https://github.com/eager-dev/eagerx/commit/efa505cd0b788ef7ac28fb69147d006564b19cc3))

## v0.1.9 (2022-03-16)
### Fix
* Refactor config, add initial docs for entities ([#125](https://github.com/eager-dev/eagerx/issues/125)) ([`34e108b`](https://github.com/eager-dev/eagerx/commit/34e108b78684cc252777196e6e2f154fcad6dc66))

### Documentation
* Create docs structure and begin with pendulum tutorial ([#124](https://github.com/eager-dev/eagerx/issues/124)) ([`6365b22`](https://github.com/eager-dev/eagerx/commit/6365b22dfdf434138f15d7af07249a31dd0903f9))

## v0.1.8 (2022-03-14)
### Fix
* Shutdown on sigint, dispose disposables, no duplicate shutdown msg ([#123](https://github.com/eager-dev/eagerx/issues/123)) ([`1bb6910`](https://github.com/eager-dev/eagerx/commit/1bb6910766c0987d6335cffba104512f5f3f9829))

## v0.1.7 (2022-03-09)
### Fix
* Test release ([#122](https://github.com/eager-dev/eagerx/issues/122)) ([`dcf00dc`](https://github.com/eager-dev/eagerx/commit/dcf00dc13f8caee63fd35f0a65029b62b2d9c160))
* Test release ([#121](https://github.com/eager-dev/eagerx/issues/121)) ([`b73928b`](https://github.com/eager-dev/eagerx/commit/b73928bc41f24b9e3ed54854ef3e789ea8eba7ee))
* Test release ([#120](https://github.com/eager-dev/eagerx/issues/120)) ([`d33e55f`](https://github.com/eager-dev/eagerx/commit/d33e55fbc928829e6f6ac035a61de4890fed6f5a))
* Test release ([#119](https://github.com/eager-dev/eagerx/issues/119)) ([`499b9d8`](https://github.com/eager-dev/eagerx/commit/499b9d8b7ecd3626395d4611d01a93391c7401c2))
* Test release ([#118](https://github.com/eager-dev/eagerx/issues/118)) ([`68e166d`](https://github.com/eager-dev/eagerx/commit/68e166d8b8699ca6c8715b2ed7d518db0d872973))
* Test release ([#117](https://github.com/eager-dev/eagerx/issues/117)) ([`b01705e`](https://github.com/eager-dev/eagerx/commit/b01705ea6af913766cb8e4fa69997401fe53cd22))
* Test release ([#116](https://github.com/eager-dev/eagerx/issues/116)) ([`e031171`](https://github.com/eager-dev/eagerx/commit/e031171e063dc5368841360581a2bc53a9a188e7))
* Refactor object api ([#114](https://github.com/eager-dev/eagerx/issues/114)) ([`26c0670`](https://github.com/eager-dev/eagerx/commit/26c06707a9c64aaf0b1aa3a35fb6c489a88be773))

## v0.1.6 (2022-03-05)
### Fix
* Argvs for executables ([#108](https://github.com/eager-dev/eagerx/issues/108)) ([`b4beb53`](https://github.com/eager-dev/eagerx/commit/b4beb539b5a2c88ad0ea801feabe324e4eb020bc))

## v0.1.5 (2022-03-04)
### Fix
* Pipeline initialization with state reset ([#107](https://github.com/eager-dev/eagerx/issues/107)) ([`529c2da`](https://github.com/eager-dev/eagerx/commit/529c2da49c3986daa99f453ab36726e922106ddb))

## v0.1.4 (2022-03-03)
### Fix
* **ci:** Remove incorrect permissions ([#106](https://github.com/eager-dev/eagerx/issues/106)) ([`eafb7bc`](https://github.com/eager-dev/eagerx/commit/eafb7bcc12390a42049d216f32a15e9ed05866b2))

## v0.1.3 (2022-03-03)
### Fix
* Test release ([`81a6d23`](https://github.com/eager-dev/eagerx/commit/81a6d23ec7b4e1b66cc6b3a753e7d83fd55a8709))
