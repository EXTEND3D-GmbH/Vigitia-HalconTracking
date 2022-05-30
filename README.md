
<picture>
  <source media="(prefers-color-scheme: light)" srcset="https://www.extend3d.com/wp-content/themes/extend3d/dist/images/LOGO.svg">
  <source media="(prefers-color-scheme: dark)" srcset="https://user-images.githubusercontent.com/74293493/170223696-34f5027b-a37a-4b78-bd5d-164381ff5b80.svg">
  <img alt="Extend3D GmbH Logo">
</picture>

# Vigitia
![Vigita-Skizze](https://user-images.githubusercontent.com/74293493/169037302-4e572fdb-9c26-44a8-b6fd-9bb88573f824.png)

Im Verbundprojekt VIGITIA untersuchen die Projektpartner, wie projizierte erweiterte Realität (projected augmented reality / PAR) Alltagsinteraktionen rund um Tische unterstützen und bereichern kann.

Bei PAR nehmen eine oder mehrere Kameras in Echtzeit die Tischoberfläche und darauf befindliche Objekte auf. Ein an der Decke oder an einem Schwenkarm befestigter Projektor kann dadurch millimetergenau und ohne Verzerrungen auf die Tischplatte bzw. die Objekte projizieren. Dies erlaubt es, zusätzliche Informationen und Interaktionsmöglichkeiten zu den Objekten einzublenden und analoge Arbeitsprozesse zu unterstützen. Parallel dazu besteht dennoch die Möglichkeit, bereits vorhandene, intelligente Gegenstände wie z.B. Smartphones, Tablets oder Laptops nahtlos in das System zu integrieren, um beispielsweise über die Projektion als Vermittler Daten zwischen den Geräten auszutauschen oder eine größere Interaktionsfläche für mehrere Personen zu bieten, als der kleine Smartphone-Bildschirm es ermöglicht. Hierbei tritt die Technik in den Hintergrund und stattdessen rückt das gemeinsame Erleben in den Vordergrund, welches durch die Technik nur auf Wunsch unterstützt wird.

# Description
This project contains code for a prototype shown in the Deggigner Exhibition which was run and tested on a Windows PC. It demonstrates simple tracking of objects that were taught using the Halcon library from MvTec. The project is split in two parts:
1. The teaching/learning phase where different models are segemented in 2D
2. The "run" phase where the taught models are found and tracked if they are in the scene.

# Code
The code was extracted out of a larger codebase due to licensing and is not complete. Stubs are used where image acquisition is needed, all tracking / teaching code is included.

To use this project a license for MvTec's Halcon must be present on the PC. Compiling should work without a license but the Halcon Library must be found by CMake (see `halcon_tracking/CMakeLists.txt:30`).

# Usage
To help segmentation and tracking two regeions of interest are defined. One is defined manually for the table where the objects should be tracked on the table. The other is the so called search region.
The idea behind the teaching/learning phase of the prototype is that a projector projects a green rectangle onto the homogenous surface where the objects is taught, this is the search region. This allows for automatic segmentation while learning different perspectives of the object. Additionally to the region of interest a color range in HSV can be applied to enhance segmentation.

Once multiple perspectives of the object were taught they can be tracked across the whole table region, unless perspective distortion won't allow a fit.

Some test data that was used (without the accompanying sound files) can be found in the `examples_files/` folder. The `state.json` describe the attributes defined in code for visible interactions on the table. The sub-folders describe a single trackable with multiple perspectives (`.sm files`) the two regions `.hobj files` and a `config.json` which describes the HSV range for this model.

