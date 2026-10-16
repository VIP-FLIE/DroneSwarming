DOT ?= dot

ARCHITECTURE_DIR := docs/architecture
GENERATED_DIR := $(ARCHITECTURE_DIR)/generated
DIAGRAMS := current-system future-system
FORMATS := svg png
DIAGRAM_OUTPUTS := $(foreach diagram,$(DIAGRAMS),$(foreach format,$(FORMATS),$(GENERATED_DIR)/$(diagram).$(format)))

.PHONY: all diagrams clean-diagrams

all: diagrams

diagrams: $(DIAGRAM_OUTPUTS)

$(GENERATED_DIR)/%.svg: $(ARCHITECTURE_DIR)/%.dot
	mkdir -p $(GENERATED_DIR)
	$(DOT) -Tsvg -Gdpi=180 -o $@ $<

$(GENERATED_DIR)/%.png: $(ARCHITECTURE_DIR)/%.dot
	mkdir -p $(GENERATED_DIR)
	$(DOT) -Tpng -Gdpi=180 -o $@ $<

clean-diagrams:
	rm -rf $(GENERATED_DIR)
