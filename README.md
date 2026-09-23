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

An aircraft mod built on Core commits its own copy of the Core headers it
includes, so it builds without Core checked out beside it. See "Building your mod
against Core's headers" in `docs/AIRCRAFT_GUIDE.md`.

## Documentation

| file | is |
|---|---|
| `docs/AIRCRAFT_GUIDE.md` | step by step: taking an aircraft from nothing to full systems, including how your mod builds against Core's headers |
| `addons/helisim/components.hpp` | field reference for components |
| `addons/helisim/controls.hpp` | field reference for switches, buttons and levers |

Start with the guide. The two `.hpp` files are the authority on what each field
means, and they sit beside the code so they stay honest.
