# Exact recovered R13 source delta

These ten files are immutable evidence from laptop capture
`DESKTOP-9LF81EF-20261025T2000`; they are not a second build tree. Apply them to the
Git base recorded in [MANIFEST.json](MANIFEST.json), with the recorded shared-core
revision, to recover the tested source delta. Active `controller/` sources also
contain later R14 changes and are not byte-identical to this evidence.

[Capture verification](verification.json) ·
[Retained binary/ELF identity](retained-binary-descriptor.json) ·
[Current firmware scope](../../FIRMWARE_STATUS.md)

The full capture, Git bundles, source ZIP, raw measurements and binaries remain in
the external handoff archive documented in the firmware status. The retained final
binary has six verification legs and a five-minute wave test. Earlier long soaks
used a subsequently overwritten binary; that distinction is preserved.
