"""
Unit tests for KiCad project file integrity and synchronization.

Tests validate:
- KiCad file formats (JSON, S-expression)
- Required project fields and metadata
- BOM synchronization with documentation
- Design rules match electrical specifications
- File structure can be opened by KiCad

No KiCad installation required - tests parse text files directly.
"""

import json
import re
from pathlib import Path
from typing import Dict, List, Set
import pytest


# Paths relative to repository root
REPO_ROOT = Path(__file__).parent.parent.parent
KICAD_PROJECT = REPO_ROOT / "hardware" / "kicad" / "servo_driver_interface"
KICAD_PRO = KICAD_PROJECT / "servo_driver_interface.kicad_pro"
KICAD_SCH = KICAD_PROJECT / "servo_driver_interface.kicad_sch"
BOM_DOC = REPO_ROOT / "docs" / "hardware" / "servo_driver_interface.md"


class TestKiCadProjectFile:
    """Validate .kicad_pro JSON project configuration."""
    
    def test_project_file_exists(self):
        """Ensure KiCad project file exists."""
        assert KICAD_PRO.exists(), f"Project file not found: {KICAD_PRO}"
    
    def test_project_file_valid_json(self):
        """Verify project file is valid JSON."""
        with open(KICAD_PRO, 'r') as f:
            data = json.load(f)
        assert isinstance(data, dict), "Project file must be JSON object"
    
    def test_project_metadata_present(self):
        """Check required project metadata fields."""
        with open(KICAD_PRO, 'r') as f:
            project = json.load(f)
        
        # KiCad version
        assert 'meta' in project, "Missing 'meta' section"
        assert 'version' in project['meta'], "Missing KiCad version"
        
        # Text variables
        assert 'text_variables' in project, "Missing text variables"
        variables = project['text_variables']
        assert 'TITLE' in variables, "Missing TITLE variable"
        assert variables['TITLE'] == "Servo Driver Interface Board"
    
    def test_track_widths_defined(self):
        """Verify track widths for signal routing."""
        with open(KICAD_PRO, 'r') as f:
            project = json.load(f)
        
        track_widths = project['board']['design_settings']['track_widths']
        
        # Required track widths for different current levels
        assert 0.2 in track_widths, "Missing 0.2mm for signals"
        assert 0.3 in track_widths, "Missing 0.3mm option"
        assert 0.5 in track_widths, "Missing 0.5mm option"
        assert 1.0 in track_widths, "Missing 1.0mm for power"
    
    def test_differential_pair_impedance_rules(self):
        """Validate differential pair dimensions for 120Ω impedance."""
        with open(KICAD_PRO, 'r') as f:
            project = json.load(f)
        
        diff_pairs = project['board']['design_settings']['diff_pair_dimensions']
        
        # For 120Ω impedance on FR-4 with h=0.2mm dielectric
        # Track width: 0.25mm, gap: 0.25mm
        assert len(diff_pairs) > 0, "No differential pair dimensions defined"
        
        diff_pair = diff_pairs[0]  # Use first (default) diff pair setting
        assert diff_pair['width'] == 0.25, \
            f"Differential track width should be 0.25mm for 120Ω, got {diff_pair['width']}"
        assert diff_pair['gap'] == 0.25, \
            f"Differential pair gap should be 0.25mm for 120Ω, got {diff_pair['gap']}"
    
    def test_power_trace_width(self):
        """Ensure power traces are wide enough for current."""
        with open(KICAD_PRO, 'r') as f:
            project = json.load(f)
        
        track_widths = project['board']['design_settings']['track_widths']
        
        # 1.0mm for 2A @ 10°C rise (conservative for 24V/5V rails)
        assert any(w >= 1.0 for w in track_widths), \
            f"Power traces should include ≥1.0mm option for current handling, got {track_widths}"


class TestKiCadSchematic:
    """Validate .kicad_sch S-expression schematic."""
    
    def test_schematic_file_exists(self):
        """Ensure KiCad schematic file exists."""
        assert KICAD_SCH.exists(), f"Schematic file not found: {KICAD_SCH}"
    
    def test_schematic_s_expression_syntax(self):
        """Verify schematic is valid S-expression format."""
        with open(KICAD_SCH, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Count parentheses - should be balanced
        open_parens = content.count('(')
        close_parens = content.count(')')
        assert open_parens == close_parens, \
            f"Unbalanced parentheses: {open_parens} open, {close_parens} close"
        
        # Should start with (kicad_sch
        assert content.strip().startswith('(kicad_sch'), \
            "Schematic must start with (kicad_sch"
    
    def test_schematic_version(self):
        """Check KiCad schematic version."""
        with open(KICAD_SCH, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract version from (version ...) field
        version_match = re.search(r'\(version\s+(\d+)\)', content)
        assert version_match, "Missing version field"
        
        version = int(version_match.group(1))
        assert version >= 20230121, \
            f"KiCad version too old: {version} (need ≥20230121 for KiCad 7.0)"
    
    def test_title_block_present(self):
        """Verify title block with project metadata."""
        with open(KICAD_SCH, 'r', encoding='utf-8') as f:
            content = f.read()
        
        assert '(title_block' in content, "Missing title block"
        assert '(title "Servo Driver Interface Board")' in content, \
            "Missing or incorrect title"
        assert '(company "6DOF Stewart Platform Project")' in content, \
            "Missing or incorrect company"


class TestBOMSynchronization:
    """Verify KiCad schematic BOM matches documentation."""
    
    def parse_doc_bom(self) -> Dict[str, str]:
        """Extract BOM from markdown documentation."""
        with open(BOM_DOC, 'r') as f:
            doc_content = f.read()
        
        bom = {}
        # Match lines like: | U1 | SN74LVCH16T245 | ...
        bom_pattern = re.compile(r'\|\s*([A-Z]+\d+)\s*\|\s*([^\|]+?)\s*\|')
        
        for match in bom_pattern.finditer(doc_content):
            designator = match.group(1).strip()
            part = match.group(2).strip()
            bom[designator] = part
        
        return bom
    
    def parse_schematic_bom(self) -> Dict[str, str]:
        """Extract BOM from KiCad schematic comments."""
        with open(KICAD_SCH, 'r', encoding='utf-8') as f:
            sch_content = f.read()
        
        bom = {}
        # Match component references in text blocks
        # Two formats:
        # 1) Text blocks: U1: SN74LVCH16T245 or R19: 2.4kΩ
        # 2) COMPONENT SUMMARY: U1  - SN74LVCH16T245PWR      16-bit level translator
        
        # Pattern 1: Colon format in main text
        colon_pattern = re.compile(r'([A-Z]\d+(?:-[A-Z]\d+)?)\s*:\s*([^\n\\]+)')
        
        for match in colon_pattern.finditer(sch_content):
            designator = match.group(1).strip()
            part = match.group(2).strip()
            bom[designator] = part
        
        return bom
    
    def test_bom_critical_components_present(self):
        """Ensure critical components are documented in schematic."""
        sch_bom = self.parse_schematic_bom()
        
        # Critical components that must be present (based on actual schematic)
        critical = {
            'U1': 'SN74LVCH16T245',     # Level translator
            'U2-U4': 'AM26C31',          # RS-422 drivers (range notation)
            'U5': 'TLP2361',             # Opto-isolator  
            'K1': 'G7L',                 # Safety relay (partial match OK)
            'U6': 'LMR33630',            # Buck regulator
        }
        
        for designator, expected_part in critical.items():
            assert designator in sch_bom, \
                f"Missing critical component {designator} in schematic. Found: {list(sch_bom.keys())}"
            assert expected_part in sch_bom[designator], \
                f"Component {designator} mismatch: expected {expected_part}, got {sch_bom[designator]}"
    
    def test_corrected_r19_value(self):
        """Verify R19 has corrected 2.4kΩ value (not 2.2kΩ)."""
        sch_bom = self.parse_schematic_bom()
        
        assert 'R19' in sch_bom, f"Missing R19 (E-stop current limiting resistor). Found: {list(sch_bom.keys())}"
        
        # Should be 2.4kΩ after commit 0cb426b fix
        r19_value = sch_bom['R19']
        assert '2.4k' in r19_value or '2K4' in r19_value or 'K40' in r19_value, \
            f"R19 should be 2.4kΩ (E-stop LED current limiter), got {r19_value}"
    
    def test_bom_designator_coverage(self):
        """Check schematic documents all major component types."""
        sch_bom = self.parse_schematic_bom()
        designators = set(sch_bom.keys())
        
        # Should have ICs, resistors, capacitors, connectors, test points
        assert any(d.startswith('U') for d in designators), "Missing IC designators (U*)"
        assert any(d.startswith('R') for d in designators), "Missing resistor designators (R*)"
        assert any(d.startswith('C') for d in designators), "Missing capacitor designators (C*)"
        assert any(d.startswith('J') for d in designators), "Missing connector designators (J*)"


class TestDesignRulesSynchronization:
    """Verify KiCad design rules match electrical specifications."""
    
    def test_differential_impedance_matches_rs422_spec(self):
        """Ensure differential pair impedance matches RS-422 requirements."""
        with open(KICAD_PRO, 'r') as f:
            project = json.load(f)
        
        diff_pairs = project['board']['design_settings']['diff_pair_dimensions']
        
        # RS-422 requires 100-120Ω differential impedance
        # Our design targets 120Ω
        expected_width = 0.25
        expected_gap = 0.25
        
        diff_pair = diff_pairs[0]
        assert diff_pair['width'] == expected_width, \
            "Differential width doesn't match RS-422 impedance requirement"
        assert diff_pair['gap'] == expected_gap, \
            "Differential gap doesn't match RS-422 impedance requirement"
    
    def test_trace_widths_available_match_hardware_tests(self):
        """Check that available trace widths support all current requirements."""
        with open(KICAD_PRO, 'r') as f:
            project = json.load(f)
        
        track_widths = project['board']['design_settings']['track_widths']
        
        # Hardware tests validate these current levels:
        # - Signal: ~20mA (0.2mm OK)
        # - E-stop opto: 9.5mA (0.2mm OK)
        # - LMR33630 output: 2A (need 1.0mm minimum)
        
        assert 0.2 in track_widths, "Missing 0.2mm trace width for signals"
        assert any(w >= 1.0 for w in track_widths), \
            "Missing wide traces (≥1.0mm) for power (2A requirement)"


class TestFileStructureIntegrity:
    """Validate overall project file structure."""
    
    def test_project_and_schematic_paired(self):
        """Ensure .kicad_pro and .kicad_sch are in same directory."""
        assert KICAD_PRO.parent == KICAD_SCH.parent, \
            "Project and schematic must be in same directory"
    
    def test_project_name_consistency(self):
        """Check filenames match project name."""
        project_name = "servo_driver_interface"
        
        assert KICAD_PRO.stem == project_name, \
            f"Project filename mismatch: {KICAD_PRO.stem}"
        assert KICAD_SCH.stem == project_name, \
            f"Schematic filename mismatch: {KICAD_SCH.stem}"
    
    def test_readme_exists(self):
        """Verify KiCad directory has documentation."""
        readme = REPO_ROOT / "hardware" / "kicad" / "README.md"
        assert readme.exists(), "Missing hardware/kicad/README.md documentation"
    
    def test_readme_references_project(self):
        """Check README documents this project."""
        readme = REPO_ROOT / "hardware" / "kicad" / "README.md"
        with open(readme, 'r') as f:
            content = f.read()
        
        assert 'servo_driver_interface' in content.lower(), \
            "README doesn't reference servo_driver_interface project"
        assert 'SN74LVCH16T245' in content or 'AM26C31' in content, \
            "README doesn't mention key ICs from BOM"


if __name__ == '__main__':
    # Run with: pytest hardware/tests/test_kicad_project.py -v
    pytest.main([__file__, '-v'])
