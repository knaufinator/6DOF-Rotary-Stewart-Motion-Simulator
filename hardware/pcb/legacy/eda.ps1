# EasyEDA Bridge PowerShell Helper
# Usage: . .\eda.ps1    (dot-source to load functions)
# Then:  eda status | eda sch-list | eda pcb-nets | eda cmd sch.wire.getAll '{}'

$script:BASE = "http://localhost:8099"

function Invoke-Eda {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory)][string]$Path,
        [string]$Method = "POST",
        [object]$Body = @{}
    )
    $uri = "$script:BASE$Path"
    try {
        if ($Method -eq "GET") {
            $r = Invoke-RestMethod -Uri $uri -Method Get -TimeoutSec 25
        } else {
            $json = if ($Body -is [string]) { $Body } else { $Body | ConvertTo-Json -Depth 10 -Compress }
            $r = Invoke-RestMethod -Uri $uri -Method Post -Body $json -ContentType "application/json" -TimeoutSec 25
        }
        return $r
    } catch {
        Write-Host "ERROR: $_" -ForegroundColor Red
        return $null
    }
}

# ===== Status =====
function Get-EdaStatus { Invoke-Eda -Path "/status" -Method GET }
function Get-EdaApi { Invoke-Eda -Path "/api" -Method GET }
function Get-EdaLog { param([int]$Limit=50) Invoke-Eda -Path "/log/events?limit=$Limit" -Method GET }
function Get-EdaMsgLog { param([int]$Limit=50) Invoke-Eda -Path "/log/messages?limit=$Limit" -Method GET }

# ===== Raw Command =====
function Send-EdaCmd {
    param(
        [Parameter(Mandatory)][string]$Method,
        [hashtable]$Params = @{}
    )
    Invoke-Eda -Path "/cmd" -Body @{ method = $Method; params = $Params }
}

# ===== Schematic: Components =====
function Get-SchComponents { param([string]$Type) Invoke-Eda -Path "/sch/components" -Body @{ type = $Type } }
function Get-SchComponent { param([Parameter(Mandatory)][string]$Id) Invoke-Eda -Path "/sch/component/get" -Body @{ id = $Id } }
function Get-SchPins { param([Parameter(Mandatory)][string]$Id) Invoke-Eda -Path "/sch/component/pins" -Body @{ id = $Id } }
function Set-SchComponent {
    param([Parameter(Mandatory)][string]$Id, [Parameter(Mandatory)][hashtable]$Props)
    Invoke-Eda -Path "/sch/component/modify" -Body @{ id = $Id; props = $Props }
}
function Move-SchComponent {
    param([Parameter(Mandatory)][string]$Id, [double]$X, [double]$Y)
    $b = @{ id = $Id }
    if ($PSBoundParameters.ContainsKey('X')) { $b.x = $X }
    if ($PSBoundParameters.ContainsKey('Y')) { $b.y = $Y }
    Invoke-Eda -Path "/sch/component/move" -Body $b
}
function Rotate-SchComponent {
    param([Parameter(Mandatory)][string]$Id, [double]$Angle = 90)
    Invoke-Eda -Path "/sch/component/rotate" -Body @{ id = $Id; angle = $Angle }
}
function Remove-SchComponent {
    param([Parameter(Mandatory)][string[]]$Ids)
    Invoke-Eda -Path "/sch/component/delete" -Body @{ ids = $Ids }
}

# ===== Schematic: Wires =====
function Get-SchWires { param([string]$Net) Invoke-Eda -Path "/sch/wires" -Body @{ net = $Net } }
function New-SchWire {
    param([double]$X1, [double]$Y1, [double]$X2, [double]$Y2, [string]$Net)
    $b = @{ line = @($X1, $Y1, $X2, $Y2) }
    if ($Net) { $b.net = $Net }
    Invoke-Eda -Path "/sch/wire/create" -Body $b
}
function Connect-SchWire {
    param([double[]]$Points, [string]$Net)
    Invoke-Eda -Path "/sch/wire/connect" -Body @{ points = $Points; net = $Net }
}
function Remove-SchWire {
    param([Parameter(Mandatory)][string[]]$Ids)
    Invoke-Eda -Path "/sch/wire/delete" -Body @{ ids = $Ids }
}

# ===== Schematic: Selection & Doc =====
function Get-SchSelection { Invoke-Eda -Path "/sch/select/all" -Body @{} }
function Clear-SchSelection { Invoke-Eda -Path "/sch/select/clear" -Body @{} }
function Select-SchPrimitive { param([string[]]$Ids) Invoke-Eda -Path "/sch/select" -Body @{ ids = $Ids } }
function Save-Sch { Invoke-Eda -Path "/sch/save" -Body @{} }
function Get-SchNetlist { param([string]$Type = "EasyEDA") Invoke-Eda -Path "/sch/netlist" -Body @{ type = $Type } }
function Invoke-SchDrc { param([switch]$Strict, [switch]$Ui) Invoke-Eda -Path "/sch/drc" -Body @{ strict = [bool]$Strict; ui = [bool]$Ui } }
function Invoke-WireAll { Invoke-Eda -Path "/sch/wire-all" -Body @{} }

# ===== Schematic: Primitives =====
function Get-SchPrimitive { param([Parameter(Mandatory)][string]$Id) Invoke-Eda -Path "/sch/primitive/get" -Body @{ id = $Id } }
function Get-SchBBox { param([Parameter(Mandatory)][string]$Id) Invoke-Eda -Path "/sch/primitive/bbox" -Body @{ id = $Id } }

# ===== PCB: Components =====
function Get-PcbComponents { param([string]$Layer) Invoke-Eda -Path "/pcb/components" -Body @{ layer = $Layer } }
function Get-PcbComponent { param([Parameter(Mandatory)][string]$Id) Invoke-Eda -Path "/pcb/component/get" -Body @{ ids = @($Id) } }
function Get-PcbPins { param([Parameter(Mandatory)][string]$Id) Invoke-Eda -Path "/pcb/component/pins" -Body @{ id = $Id } }
function Move-PcbComponent {
    param([Parameter(Mandatory)][string]$Id, [double]$X, [double]$Y)
    $b = @{ id = $Id }
    if ($PSBoundParameters.ContainsKey('X')) { $b.x = $X }
    if ($PSBoundParameters.ContainsKey('Y')) { $b.y = $Y }
    Invoke-Eda -Path "/pcb/component/move" -Body $b
}
function Rotate-PcbComponent {
    param([Parameter(Mandatory)][string]$Id, [double]$Angle = 90)
    Invoke-Eda -Path "/pcb/component/rotate" -Body @{ id = $Id; angle = $Angle }
}
function Flip-PcbComponent {
    param([Parameter(Mandatory)][string]$Id, [string]$Layer = "BottomLayer")
    Invoke-Eda -Path "/pcb/component/flip" -Body @{ id = $Id; layer = $Layer }
}
function Set-PcbComponent {
    param([Parameter(Mandatory)][string]$Id, [Parameter(Mandatory)][hashtable]$Props)
    Invoke-Eda -Path "/pcb/component/modify" -Body @{ id = $Id; props = $Props }
}

# ===== PCB: Nets =====
function Get-PcbNets { Invoke-Eda -Path "/pcb/nets" -Body @{} }
function Get-PcbNetPrimitives { param([Parameter(Mandatory)][string]$Net, [string[]]$Types) Invoke-Eda -Path "/pcb/net/primitives" -Body @{ net = $Net; types = $Types } }
function Get-PcbNetLength { param([Parameter(Mandatory)][string]$Net) Invoke-Eda -Path "/pcb/net/length" -Body @{ net = $Net } }
function Highlight-PcbNet { param([Parameter(Mandatory)][string]$Net) Invoke-Eda -Path "/pcb/net/highlight" -Body @{ net = $Net } }

# ===== PCB: Tracks & Vias =====
function Get-PcbTracks { param([string]$Net, [string]$Layer) Invoke-Eda -Path "/pcb/lines" -Body @{ net = $Net; layer = $Layer } }
function New-PcbTrack {
    param([string]$Net, [string]$Layer, [double]$X1, [double]$Y1, [double]$X2, [double]$Y2, [double]$Width)
    Invoke-Eda -Path "/pcb/line/create" -Body @{ net=$Net; layer=$Layer; startX=$X1; startY=$Y1; endX=$X2; endY=$Y2; lineWidth=$Width }
}
function Get-PcbVias { param([string]$Net) Invoke-Eda -Path "/pcb/vias" -Body @{ net = $Net } }
function New-PcbVia {
    param([string]$Net, [double]$X, [double]$Y, [double]$HoleDia, [double]$Dia)
    Invoke-Eda -Path "/pcb/via/create" -Body @{ net=$Net; x=$X; y=$Y; holeDiameter=$HoleDia; diameter=$Dia }
}

# ===== PCB: Pads & Layers =====
function Get-PcbPads { param([string]$Layer, [string]$Net) Invoke-Eda -Path "/pcb/pads" -Body @{ layer=$Layer; net=$Net } }
function Get-PcbLayers { Invoke-Eda -Path "/pcb/layers" -Body @{} }
function Select-PcbLayer { param([Parameter(Mandatory)][string]$Layer) Invoke-Eda -Path "/pcb/layer/select" -Body @{ layer = $Layer } }

# ===== PCB: DRC =====
function Invoke-PcbDrc { param([switch]$Strict,[switch]$Ui,[switch]$Verbose) Invoke-Eda -Path "/pcb/drc/check" -Body @{ strict=[bool]$Strict; ui=[bool]$Ui; verbose=[bool]$Verbose } }
function Get-PcbDrcRules { Invoke-Eda -Path "/pcb/drc/rules" -Body @{} }
function Get-PcbNetClasses { Invoke-Eda -Path "/pcb/drc/netClasses" -Body @{} }

# ===== PCB: Document =====
function Save-Pcb { param([string]$Uuid) Invoke-Eda -Path "/pcb/save" -Body @{ uuid = $Uuid } }
function Move-PcbView { param([double]$X, [double]$Y) Invoke-Eda -Path "/pcb/navigateTo" -Body @{ x=$X; y=$Y } }
function Get-PcbOrigin { Invoke-Eda -Path "/pcb/origin" -Body @{} }
function Zoom-PcbBoard { Invoke-Eda -Path "/pcb/zoomToBoard" -Body @{} }
function Get-PcbAtPoint { param([double]$X, [double]$Y) Invoke-Eda -Path "/pcb/getAtPoint" -Body @{ x=$X; y=$Y } }

# ===== Library =====
function Search-EdaLib { param([Parameter(Mandatory)][string]$Query, [int]$Limit=10) Invoke-Eda -Path "/lib/search" -Body @{ query=$Query; limit=$Limit } }
function Get-EdaDevice { param([Parameter(Mandatory)][string]$Uuid, [string]$LibUuid) Invoke-Eda -Path "/lib/device" -Body @{ uuid=$Uuid; libraryUuid=$LibUuid } }
function Get-EdaLcsc { param([Parameter(Mandatory)][string[]]$Ids) Invoke-Eda -Path "/lib/lcsc" -Body @{ ids=$Ids } }
function Get-EdaLibraries { Invoke-Eda -Path "/lib/all" -Body @{} }

# ===== Batch =====
function Invoke-SchBatch {
    param([Parameter(Mandatory)][hashtable[]]$Commands, [switch]$StopOnError)
    Invoke-Eda -Path "/sch/batch" -Body @{ commands = $Commands; stopOnError = [bool]$StopOnError }
}
function Invoke-PcbBatch {
    param([Parameter(Mandatory)][hashtable[]]$Commands, [switch]$StopOnError)
    Invoke-Eda -Path "/pcb/batch" -Body @{ commands = $Commands; stopOnError = [bool]$StopOnError }
}

# ===== Convenience: eda command =====
function eda {
    [CmdletBinding()]
    param([Parameter(Position=0)][string]$Action, [Parameter(ValueFromRemainingArguments)]$Args)
    switch ($Action) {
        "status"     { Get-EdaStatus }
        "api"        { Get-EdaApi }
        "log"        { Get-EdaLog @Args }
        "msglog"     { Get-EdaMsgLog @Args }
        "cmd"        { Send-EdaCmd -Method $Args[0] -Params $(if($Args[1]){$Args[1] | ConvertFrom-Json -AsHashtable}else{@{}}) }
        # Schematic
        "sch-list"   { Get-SchComponents @Args }
        "sch-get"    { Get-SchComponent -Id $Args[0] }
        "sch-pins"   { Get-SchPins -Id $Args[0] }
        "sch-move"   { Move-SchComponent -Id $Args[0] -X $Args[1] -Y $Args[2] }
        "sch-rotate" { Rotate-SchComponent -Id $Args[0] -Angle $(if($Args[1]){$Args[1]}else{90}) }
        "sch-modify" { Set-SchComponent -Id $Args[0] -Props ($Args[1] | ConvertFrom-Json -AsHashtable) }
        "sch-wires"  { Get-SchWires @Args }
        "sch-wire"   { New-SchWire -X1 $Args[0] -Y1 $Args[1] -X2 $Args[2] -Y2 $Args[3] -Net $Args[4] }
        "sch-sel"    { Get-SchSelection }
        "sch-save"   { Save-Sch }
        "sch-drc"    { Invoke-SchDrc -Ui }
        "sch-netlist"{ Get-SchNetlist }
        "wire-all"   { Invoke-WireAll }
        # PCB
        "pcb-list"   { Get-PcbComponents @Args }
        "pcb-get"    { Get-PcbComponent -Id $Args[0] }
        "pcb-pins"   { Get-PcbPins -Id $Args[0] }
        "pcb-move"   { Move-PcbComponent -Id $Args[0] -X $Args[1] -Y $Args[2] }
        "pcb-rotate" { Rotate-PcbComponent -Id $Args[0] -Angle $(if($Args[1]){$Args[1]}else{90}) }
        "pcb-flip"   { Flip-PcbComponent -Id $Args[0] }
        "pcb-nets"   { Get-PcbNets }
        "pcb-tracks" { Get-PcbTracks @Args }
        "pcb-vias"   { Get-PcbVias @Args }
        "pcb-layers" { Get-PcbLayers }
        "pcb-drc"    { Invoke-PcbDrc -Ui }
        "pcb-save"   { Save-Pcb }
        "pcb-zoom"   { Zoom-PcbBoard }
        # Library
        "lib-search" { Search-EdaLib -Query $Args[0] }
        "lib-lcsc"   { Get-EdaLcsc -Ids $Args }
        "lib-list"   { Get-EdaLibraries }
        default {
            Write-Host "EasyEDA Bridge CLI" -ForegroundColor Cyan
            Write-Host ""
            Write-Host "STATUS:" -ForegroundColor Yellow
            Write-Host "  eda status              Connection status & stats"
            Write-Host "  eda api                 List all API routes"
            Write-Host "  eda log [n]             Event log (last n entries)"
            Write-Host "  eda msglog [n]          Message log (TX/RX)"
            Write-Host "  eda cmd <method> [json] Raw API command"
            Write-Host ""
            Write-Host "SCHEMATIC:" -ForegroundColor Yellow
            Write-Host "  eda sch-list [type]     List components (part/netflag/netport/netlabel)"
            Write-Host "  eda sch-get <id>        Get component details"
            Write-Host "  eda sch-pins <id>       Get component pins"
            Write-Host "  eda sch-move <id> <x> <y>  Move component"
            Write-Host "  eda sch-rotate <id> [deg]  Rotate component"
            Write-Host "  eda sch-modify <id> <json> Modify properties"
            Write-Host "  eda sch-wires [net]     List wires"
            Write-Host "  eda sch-wire <x1> <y1> <x2> <y2> [net]  Create wire"
            Write-Host "  eda sch-sel             Get selection"
            Write-Host "  eda sch-save            Save schematic"
            Write-Host "  eda sch-drc             Run DRC"
            Write-Host "  eda sch-netlist         Get netlist"
            Write-Host "  eda wire-all            Wire all predefined nets"
            Write-Host ""
            Write-Host "PCB:" -ForegroundColor Yellow
            Write-Host "  eda pcb-list [layer]    List components"
            Write-Host "  eda pcb-get <id>        Get component"
            Write-Host "  eda pcb-pins <id>       Get component pins"
            Write-Host "  eda pcb-move <id> <x> <y>  Move component"
            Write-Host "  eda pcb-rotate <id> [deg]  Rotate component"
            Write-Host "  eda pcb-flip <id>       Flip to bottom layer"
            Write-Host "  eda pcb-nets            List all net names"
            Write-Host "  eda pcb-tracks [net]    List tracks"
            Write-Host "  eda pcb-vias [net]      List vias"
            Write-Host "  eda pcb-layers          List layers"
            Write-Host "  eda pcb-drc             Run DRC"
            Write-Host "  eda pcb-save            Save PCB"
            Write-Host "  eda pcb-zoom            Zoom to board"
            Write-Host ""
            Write-Host "LIBRARY:" -ForegroundColor Yellow
            Write-Host "  eda lib-search <query>  Search component library"
            Write-Host "  eda lib-lcsc <id> ...   Lookup LCSC part numbers"
            Write-Host "  eda lib-list            List all libraries"
            Write-Host ""
            Write-Host "POWERSHELL FUNCTIONS:" -ForegroundColor Yellow
            Write-Host "  Get-SchComponents, Move-SchComponent, New-SchWire,"
            Write-Host "  Get-PcbComponents, Move-PcbComponent, Rotate-PcbComponent,"
            Write-Host "  Search-EdaLib, Get-EdaLcsc, Invoke-SchBatch, etc."
            Write-Host "  Use Get-Command *Sch* or Get-Command *Pcb* to see all"
        }
    }
}

# Export aliases
Set-Alias -Name eda-status -Value Get-EdaStatus
Set-Alias -Name eda-api -Value Get-EdaApi

Write-Host ""
Write-Host "  EasyEDA Bridge CLI loaded" -ForegroundColor Cyan
Write-Host "  Type 'eda' for help, 'eda status' to check connection" -ForegroundColor DarkGray
Write-Host ""
