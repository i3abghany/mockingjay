# Mockingjay (CMPT 750 Project Repository)

The git repository is backed up on Github and can be found [https://github.com/i3abghany/mockingjay](here). We have sent access invitations to instructors and TAs. (Professor Alaa and professor Arrvindh were granted access a month ago but the invitation expired, and we can't send new invitations without accepting/rejecting the sent invitations). If there's a problem with accessing the project, please let us know.

Authors:

Mahmoud Abumandour: mam47@sfu.ca
Marzieh Barkhordar: mba186@sfu.ca

This is our implementation of the Mockingjay eviction policy for the CMPT-750 final project. We implement it and evaluate it in Gem5. The Git repository includes a "submodule" link to our own Gem5 fork that has the policy implemented and set up. Also, we modified the default configurations files for command line options (`gem5/configs/common/Options.py`, `gem5/configs/common/Options.py`) to include the L2 cache replacement policy as an option for the `gem5/configs/example/se.py` gem5 configuration file. Hence, you can clone the [https://github.com/i3abghany/gem5/tree/2d1dd4753a6e3346b708c416956eb306ac168625](i3abghany/gem5) and it will have the replacement policy installed and runnable using the sample script in the end of this README file.

## The Presentation

The video presentation is uploaded to the github repository and is included in the `Presentation` folder. The video is also backed up on Google Drive and can be found [https://drive.google.com/file/d/1rvSLdWQ5ZTzfiGB1Ujso1xOpR0_B2rQj/view?usp=sharing](here).

## Manually adding Mockingjay to your gem5 source code

To add a new cache replacement policy to your version of gem5 do the following. This is only tested on the last version of gem5 and it may not be compatible with older versions.

1. copy the `mj_rp.hh` and `mj_rp.cc` files into `gem5/src/mem/cache/replacement_policies`
2. Add the following class to `gem5/src/mem/cache/replacement_policies/ReplacementPolicies.py`:
```python
class MJRP(BaseReplacementPolicy):
    type = "MJRP"
    cxx_class = 'gem5::replacement_policy::MJRP'
    cxx_header = "mem/cache/replacement_policies/mj_rp.hh"
```
3. In the `gem5/src/mem/cache/replacement_policies/SConscript` file, add the `MJRP` entry in the `sim_objects` of the `SimObject`call, and add `Source('mj_rp.cc')` as a separate line.

To add L2 replacement policy as a command line argument for configuration scripts, do the following steps.

1. Add this line in the `configs/common/Options.py`:
```python
parser.add_argument("--l2_repl", default="LRURP", choices=ObjectList.rp_list.get_names(), help = "replacement policy for l2")
```
2. Add this line in the `configs/common/CacheConfig.py`:
```python
system.l2.replacement_policy = ObjectList.rp_list.get(options.l2_repl)()
```

Now gem5 can recognize Mockingjay as a replacement policy that can be set to cache structures.

After building gem5, a sample script to run simulation using Mockingjay in L2 would be as follows:

```console
gem5/build/X86/gem5.fast -r -e --outdir=mj_output  ~/750/mockingjay/gem5/configs/example/se.py --cmd="path/to/example/x86/binary" -F 100000000 --warmup-insts=100000000 -I 100000000 --cpu-type="O3CPU" --caches --l2cache --l2_size "1MB" --l1d_size="32kB" --l1i_size="32kB" --l2_assoc=16 --l2_repl="MJRP"
```
We used three SPECCPU2017 benchmarks. 

* 600.perlbench_s
* 602.gcc_s
* 605.mcf_s
