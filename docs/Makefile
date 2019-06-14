ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
TEX_DIR:=$(ROOT_DIR)/tex

all: overview.pdf

README.md: pandoc-source.md  $(TEX_DIR)/citations.bib
	pandoc $< -t gfm -F pandoc-citeproc --bibliography $(word 2,$^) --csl $(TEX_DIR)/ieee.csl -f markdown+smart -o $@

overview.pdf : pandoc-source.md  $(TEX_DIR)/citations.bib
	pandoc $< -F pandoc-citeproc --bibliography $(word 2,$^) --csl $(TEX_DIR)/ieee.csl -f markdown+smart -o $@

clean:
	rm -f *.aux *.tex *.log *.bbl *.blg *.out *.bcf *.xml

.PHONY: all clean documentation
