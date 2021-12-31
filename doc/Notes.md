# Design descisions

Why I've chosen GNU Assembler over AVRA?

* GNU Assembler supports generating object files from source files and link later.
* AVRA only supports '.include' directive, which can become cumbersome when more than one file use the same source file
https://www.avrfreaks.net/forum/multiple-source-files

