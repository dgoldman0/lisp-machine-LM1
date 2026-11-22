// ============================================================================
// link_cases.svh — the link conformance cases, read from tb/link/link_cases.txt
//
// Included inside a bench module.  run_link_cases reads the case list at
// run time and performs each operation through the bench's own tasks, so
// every bench, and the chiplet module's tests (emu/tests/test_chiplet_link.py),
// run the one list.  The bench supplies check() and these tasks, one for
// each operation the list's header describes:
//
//   lc_reset, lc_identity(i, v), lc_image(n, v), lc_release(len, arg, lent),
//   lc_boot(failed), lc_boot_arg(v), lc_loaded(n, v), lc_requests(n), lc_wait,
//   lc_runtime_ready, lc_runtime_stop(b), lc_ready(b), lc_stopped(b),
//   lc_stop(b), lc_lend(i, base, limit, w),
//   lc_store(units, zeros, addr, seed, code), lc_ring_in(src),
//   lc_rung_in(src), lc_ring_out(dst), lc_rung_out(dst), lc_no_ring_out
//
// Each task calls lc_check(what, cond), which labels the check with the
// case and the line of the list it came from.
// ============================================================================

    string       lc_case = "";
    int          lc_line = 0;
    int          lc_cases_run = 0;

    task automatic lc_check(input string what, input logic cond);
        check($sformatf("%s (link_cases.txt:%0d): %s", lc_case, lc_line, what), cond);
    endtask

    // Word w of the image a case's store sends.
    function automatic logic [63:0] lc_image_word(input logic [7:0] seed, input int w);
        return {seed, 8'(w), 16'h7ACC, 32'(w * 7 + 1)};
    endfunction

    // The line without its comment.
    function automatic string lc_strip(input string line);
        for (int i = 0; i < line.len(); i++)
            if (line.getc(i) == "#")
                return (i == 0) ? "" : line.substr(0, i - 1);
        return line;
    endfunction

    task automatic run_link_cases(input string path);
        int          fd, got;
        string       line, op, s0, s1;
        logic [63:0] a0, a1, a2, a3;
        fd = $fopen(path, "r");
        if (fd == 0) begin
            check($sformatf("the case list %s opens", path), 1'b0);
            return;
        end
        lc_line = 0;
        while (!$feof(fd)) begin
            line = "";
            got = $fgets(line, fd);
            if (got == 0)
                break;
            lc_line++;
            line = lc_strip(line);
            op = "";
            if ($sscanf(line, "%s", op) < 1 || op == "")
                continue;
            a0 = '0; a1 = '0; a2 = '0; a3 = '0; s0 = ""; s1 = "";
            if (op == "case") begin
                got = $sscanf(line, "%s %s", op, s0);
                lc_case = s0;
                lc_cases_run++;
                $display("\nLink case %s", line.substr(5, line.len() - 1));
            end else if (op == "reset")          lc_reset();
            else if (op == "identity") begin
                got = $sscanf(line, "%s %h %h", op, a0, a1);
                lc_identity(a0[7:0], a1);
            end else if (op == "image") begin
                got = $sscanf(line, "%s %h %h", op, a0, a1);
                lc_image(int'(a0), a1);
            end else if (op == "release" || op == "release_unlent") begin
                got = $sscanf(line, "%s %h %h", op, a0, a1);
                lc_release(a0, a1, op == "release");
            end else if (op == "boot") begin
                got = $sscanf(line, "%s %s", op, s0);
                lc_boot(s0 == "failed");
            end else if (op == "boot_arg") begin
                got = $sscanf(line, "%s %h", op, a0);
                lc_boot_arg(a0);
            end else if (op == "loaded") begin
                got = $sscanf(line, "%s %h %h", op, a0, a1);
                lc_loaded(int'(a0), a1);
            end else if (op == "requests") begin
                got = $sscanf(line, "%s %h", op, a0);
                lc_requests(int'(a0));
            end else if (op == "wait")           lc_wait();
            else if (op == "runtime_ready")      lc_runtime_ready();
            else if (op == "runtime_stop" || op == "ready" || op == "stopped" ||
                     op == "stop") begin
                got = $sscanf(line, "%s %h", op, a0);
                if (op == "runtime_stop")        lc_runtime_stop(a0[0]);
                else if (op == "ready")          lc_ready(a0[0]);
                else if (op == "stopped")        lc_stopped(a0[0]);
                else                             lc_stop(a0[0]);
            end else if (op == "lend") begin
                got = $sscanf(line, "%s %h %h %h %h", op, a0, a1, a2, a3);
                lc_lend(a0[7:0], a1, a2, a3[0]);
            end else if (op == "store") begin
                got = $sscanf(line, "%s %s %h %h %s", op, s0, a0, a1, s1);
                if (s0 == "u8")
                    lc_store(4, 0, a0, a1[7:0], (s1 == "ok") ? 2'd0 : 2'd1);
                else if (s0 == "fp32")
                    lc_store(2, 2, a0, a1[7:0], (s1 == "ok") ? 2'd0 : 2'd1);
                else if (s0 == "fp64")
                    lc_store(1, 3, a0, a1[7:0], (s1 == "ok") ? 2'd0 : 2'd1);
                else
                    lc_check($sformatf("a store of format %s", s0), 1'b0);
            end else if (op == "ring_in" || op == "rung_in" || op == "ring_out" ||
                         op == "rung_out") begin
                got = $sscanf(line, "%s %h", op, a0);
                if (op == "ring_in")             lc_ring_in(a0[7:0]);
                else if (op == "rung_in")        lc_rung_in(a0[7:0]);
                else if (op == "ring_out")       lc_ring_out(a0[7:0]);
                else                             lc_rung_out(a0[7:0]);
            end else if (op == "no_ring_out")    lc_no_ring_out();
            else
                lc_check($sformatf("no such operation %s", op), 1'b0);
        end
        $fclose(fd);
        check($sformatf("the case list held cases (%0d)", lc_cases_run), lc_cases_run >= 12);
    endtask
