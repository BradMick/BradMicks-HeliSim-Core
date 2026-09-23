# BMKHS HeliSim Core

An airframe-agnostic helicopter flight model and systems simulation for Arma 3.

Core provides the model and the functions; it knows no aircraft. An aircraft is
built by shipping a companion pack that declares what that airframe has - its
rotor, engines, components and cockpit controls - and Core reads those
declarations. Adding a third generator is a config change, not a code change.

## Building

    hemtt build      # dev build, produces .hemttout/build/addons/bmkhs_helisim.pbo
    hemtt release    # signed release archive

Depends only on vanilla Arma and CBA. No mod dependencies.

## Documentation

| file | is |
|---|---|
| `docs/AIRCRAFT_GUIDE.md` | step by step: taking an aircraft from nothing to full systems |
| `docs/SYSTEMS.md` | the systems model, why it is shaped this way, and what bit us |
| `addons/helisim/components.hpp` | field reference for components |
| `addons/helisim/controls.hpp` | field reference for switches, buttons and levers |

Start with the guide. The two `.hpp` files are the authority on what each field
means, and they sit beside the code so they stay honest.
