############################################################
# rsp_e32_palette.S
#
# Builds the bone matrix palette on the RSP. For every bone it
# produces the two products the magma matrices uniform wants,
# model-view-projection and model-view, plus the normal matrix,
# and writes the whole uniform straight to RDRAM. The CPU only
# hands over the two per-mesh matrices and the bone array.
#
# Nothing here touches magma. The bytes written are the same
# ones 'mesh_updatePalette' used to compose on the CPU, in the
# same buffer, and magma reads them exactly as before.
#
# The multiply routine below is a verbatim copy of GL_MtxMult
# from libdragon (src/GL/rsp_gl_common.inc, Dennis Heinze,
# public domain). Its matrix layout, sixteen integer parts
# followed by sixteen fractional parts, is already the layout
# of mgfx_matrix_t, so nothing had to be repacked.
############################################################

#include <rsp_queue.inc>

    .data

    RSPQ_BeginOverlayHeader
        RSPQ_DefineCommand PaletteCmd_Update, 16
    RSPQ_EndOverlayHeader

    .align 4
    .ascii "Volcano64 palett"

    RSPQ_BeginSavedState
PALETTE_STATE_UNUSED:   .long 0
    RSPQ_EndSavedState

    # Scratch for one command. Nothing here has to survive an overlay
    # switch, so it stays out of the saved state.
    .align 4
PALETTE_SCALE:  .half 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF
                .half 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF

    # These two are loaded by a single transfer, so they must stay
    # adjacent and in this order.
    .align 4
PALETTE_VM:     .ds.b 64        # view * model
PALETTE_VPM:    .ds.b 64        # view_projection * model

    .align 4
PALETTE_BONE:   .ds.b 64        # bone matrix, model space

    # One mgfx_matrices_t: mvp, mv, normal.
    .align 4
PALETTE_OUT:    .ds.b 160

    .text

    #############################################################
    # PaletteCmd_Update
    #
    # ARGS:
    #   a0: bone count (low bits of the command word)
    #   a1: RDRAM address of the two per-mesh matrices, view-model
    #       first and view-projection-model second, back to back
    #   a2: RDRAM address of the bone matrices, one mgfx_matrix_t each
    #   a3: RDRAM address of the destination palette
    #############################################################
    .func PaletteCmd_Update
PaletteCmd_Update:
    #define count   t4
    #define src     t5
    #define dst     t6
    #define vscale  $v14
    #define vnrm0_i $v15
    #define vnrm1_i $v16
    #define vnrm0_f $v17
    #define vnrm1_f $v18
    #define vnrm    $v19

    andi count, a0, 0xFFFF
    move src, a2
    move dst, a3

    # Both per-mesh matrices come in one transfer.
    move s0, a1
    li s4, %lo(PALETTE_VM)
    jal DMAIn
    li t0, DMA_SIZE(128, 1)

    # The normal matrix is the model-view scaled into s0.15, so the
    # factor is loaded once and kept for the whole palette.
    li s6, %lo(PALETTE_SCALE)
    lqv vscale, 0x00,s6

    beqz count, PaletteUpdateEnd
    nop

PaletteUpdateLoop:
    move s0, src
    li s4, %lo(PALETTE_BONE)
    jal DMAIn
    li t0, DMA_SIZE(64, 1)

    # mvp = view_projection_model x bone
    li s5, %lo(PALETTE_VPM)
    li s6, %lo(PALETTE_BONE)
    jal Palette_MtxMult
    li s7, %lo(PALETTE_OUT)

    # mv = view_model x bone
    li s5, %lo(PALETTE_VM)
    li s6, %lo(PALETTE_BONE)
    jal Palette_MtxMult
    li s7, %lo(PALETTE_OUT + 64)

    # normal = mv scaled by 0x7FFF, which saturates 1.0 to 0x7FFF
    # exactly like the CPU path did.
    li s6, %lo(PALETTE_OUT + 64)
    li s7, %lo(PALETTE_OUT + 128)
    lqv vnrm0_i, 0x00,s6
    lqv vnrm1_i, 0x10,s6
    lqv vnrm0_f, 0x20,s6
    lqv vnrm1_f, 0x30,s6

    vmudn vnrm, vnrm0_f, vscale.e0
    vmadh vnrm, vnrm0_i, vscale.e0
    sqv vnrm, 0x00,s7

    vmudn vnrm, vnrm1_f, vscale.e0
    vmadh vnrm, vnrm1_i, vscale.e0
    sqv vnrm, 0x10,s7

    move s0, dst
    li s4, %lo(PALETTE_OUT)
    jal DMAOut
    li t0, DMA_SIZE(160, 1)

    addi src, 64
    addi dst, 160
    addi count, -1
    bgtz count, PaletteUpdateLoop
    nop

PaletteUpdateEnd:
    j RSPQ_Loop
    nop

    #undef count
    #undef src
    #undef dst
    .endfunc

    #############################################################
    # Palette_MtxMult
    #
    # Verbatim from libdragon's GL_MtxMult. Computes dst = rhs x lhs
    # in the convention of fm_mat4_mul, that is, calling it with
    # lhs = a and rhs = b matches matrix4_product(dst, a, b).
    #
    # ARGS:
    #   s5: lhs address in DMEM
    #   s6: rhs address in DMEM
    #   s7: destination address in DMEM
    #############################################################
    .func Palette_MtxMult
Palette_MtxMult:
    #define lhs s5
    #define rhs s6
    #define dst s7

    #define v___         $v01

    #define vrhs01_i     $v02
    #define vrhs01_f     $v03
    #define vrhs23_i     $v04
    #define vrhs23_f     $v05

    #define vlhs0_i      $v06
    #define vlhs0_f      $v07
    #define vlhs1_i      $v08
    #define vlhs1_f      $v09
    #define vlhs2_i      $v10
    #define vlhs2_f      $v11
    #define vlhs3_i      $v12
    #define vlhs3_f      $v13

    lqv vrhs01_i, 0x00,rhs
    lqv vrhs23_i, 0x10,rhs
    lqv vrhs01_f, 0x20,rhs
    lqv vrhs23_f, 0x30,rhs

    ldv vlhs0_i.e0,  0x00,lhs
    ldv vlhs0_i.e4,  0x00,lhs
    ldv vlhs1_i.e0,  0x08,lhs
    ldv vlhs1_i.e4,  0x08,lhs
    ldv vlhs2_i.e0,  0x10,lhs
    ldv vlhs2_i.e4,  0x10,lhs
    ldv vlhs3_i.e0,  0x18,lhs
    ldv vlhs3_i.e4,  0x18,lhs
    ldv vlhs0_f.e0,  0x20,lhs
    ldv vlhs0_f.e4,  0x20,lhs
    ldv vlhs1_f.e0,  0x28,lhs
    ldv vlhs1_f.e4,  0x28,lhs
    ldv vlhs2_f.e0,  0x30,lhs
    ldv vlhs2_f.e4,  0x30,lhs
    ldv vlhs3_f.e0,  0x38,lhs
    ldv vlhs3_f.e4,  0x38,lhs

    vmudl v___,     vlhs0_f, vrhs01_f.h0
    vmadm v___,     vlhs0_i, vrhs01_f.h0
    vmadn v___,     vlhs0_f, vrhs01_i.h0
    vmadh v___,     vlhs0_i, vrhs01_i.h0

    vmadl v___,     vlhs1_f, vrhs01_f.h1
    vmadm v___,     vlhs1_i, vrhs01_f.h1
    vmadn v___,     vlhs1_f, vrhs01_i.h1
    vmadh v___,     vlhs1_i, vrhs01_i.h1

    vmadl v___,     vlhs2_f, vrhs01_f.h2
    vmadm v___,     vlhs2_i, vrhs01_f.h2
    vmadn v___,     vlhs2_f, vrhs01_i.h2
    vmadh v___,     vlhs2_i, vrhs01_i.h2

    vmadl v___,     vlhs3_f, vrhs01_f.h3
    vmadm v___,     vlhs3_i, vrhs01_f.h3
    vmadn vrhs01_f, vlhs3_f, vrhs01_i.h3
    vmadh vrhs01_i, vlhs3_i, vrhs01_i.h3

    vmudl v___,     vlhs0_f, vrhs23_f.h0
    vmadm v___,     vlhs0_i, vrhs23_f.h0
    vmadn v___,     vlhs0_f, vrhs23_i.h0
    vmadh v___,     vlhs0_i, vrhs23_i.h0

    vmadl v___,     vlhs1_f, vrhs23_f.h1
    vmadm v___,     vlhs1_i, vrhs23_f.h1
    vmadn v___,     vlhs1_f, vrhs23_i.h1
    vmadh v___,     vlhs1_i, vrhs23_i.h1

    vmadl v___,     vlhs2_f, vrhs23_f.h2
    vmadm v___,     vlhs2_i, vrhs23_f.h2
    vmadn v___,     vlhs2_f, vrhs23_i.h2
    vmadh v___,     vlhs2_i, vrhs23_i.h2

    vmadl v___,     vlhs3_f, vrhs23_f.h3
    vmadm v___,     vlhs3_i, vrhs23_f.h3
    vmadn vrhs23_f, vlhs3_f, vrhs23_i.h3
    vmadh vrhs23_i, vlhs3_i, vrhs23_i.h3

    sqv vrhs01_i, 0x00,dst
    sqv vrhs23_i, 0x10,dst
    sqv vrhs01_f, 0x20,dst
    jr ra
    sqv vrhs23_f, 0x30,dst

    #undef lhs
    #undef rhs
    #undef dst
    .endfunc
