package whaleboot

#System: {
    files: #FileHierarchy
    bootloader: #Bootloader
}

#FileHierarchy: {
    manifest: bool
    dockerenv: bool
    etc: #EtcFiles
}

#EtcFiles: {
    hostname: #File
    hosts: #File
    resolv.conf: #File
}

#File: string | #LineList | #FileSpec

// TODO: inverse-match newline (should be none in linelist strings)
#LineList: [string...]
#FileSpec: {
    type: "file" | "symlink"
    // TODO: make/import a "#Path" type...
    path: string
}

#Bootloader: {
    program: "grub" | "extlinux"
    type: "bios" | "efi" | "hybrid"
    efi-partition: {
	name: string
    }
    root-partition: {
	name: string
    }
}

#GrubBootloader: {
    program: "grub"
    grub.cfg: string | #LineList | #FileSpec | #GrubCfg
}

#EfiBootloader: {
    type: "efi"
    efi-partition: {
	name: string
    }
    root-partition: {
	name:string
    }
    efi-bin-path: string
    install-path: string
}

#BiosBootloader: {
    type: "bios"
    root-partition: {
	name:string
    }
    mbr-bin-path: string
}

#HybridBootloader:
