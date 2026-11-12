"""Build the finalized Team Plan workbook (final-term deliverable 4).

    python3 tools/make_team_plan.py        (needs openpyxl)

Writes docs/plan/Team_Plan.xlsx with five sheets: Summary (formulas over
the task list), Tasks, Review response, Timeline and Deliverables. The task
list below is the single source: edit it here and run the script again
rather than editing the workbook by hand. "Owner confirmed" stays "Pending"
until that member has read the change and said yes.
"""
import os

from openpyxl import Workbook
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter
from openpyxl.worksheet.datavalidation import DataValidation

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'plan',
                   'Team_Plan.xlsx')
MEMBERS = [('Husan', 'M1-A', 'Passenger detection and queue counting (S1)'),
           ('Tuymurod', 'M1-B', 'AI demand prediction (S2)'),
           ('Fazliddin', 'M2-A', 'Bus manager and docking (S3)'),
           ('Arkar', 'M2-B', 'Hub coordinator, crossing arbiter, S8 integration'),
           ('Carla', 'M3', 'Parking and traffic (S4, S5)'),
           ('Aidil', 'M4', 'Routing and robot escort (S6, S7)'),
           ('Saidumar', 'M5', 'Simulator, safety, infrastructure, integration (S9); team leader')]
DONE, DOING, TODO = 'Done', 'In progress', 'To do'
YES, PENDING, NA = 'Yes', 'Pending', 'n/a'

# id, workstream, task, owner, reviewer, start, due, status, owner confirmed, result / evidence
TASKS = [
    ('F01', 'User interface', 'Hub Control Center: web UI to choose, run and watch any task or sub-task '
     '(task list with options, plain-language Ask box, live map, status, verdict, history)',
     'Saidumar', 'all', '2026-11-11', '2026-11-12', DONE, YES,
     'docker/hub.sh ui; S4 11/11 and S9 13/13 started from the page; 27 unit tests'),
    ('F02', 'User interface', 'Each member checks that their own task runs from the UI with its options',
     'all', 'Saidumar', '2026-11-13', '2026-11-16', TODO, PENDING, 'checklist in the group chat'),
    ('F03', 'Review: prediction', 'Publish the future-queue forecast (assignment target) and the '
     'unserved-demand forecast as two named topics; score each on its own label',
     'Tuymurod', 'Husan', '2026-11-12', '2026-11-12', DONE, PENDING,
     '/hub/demand: ridge 1.6245 vs persistence 1.6684, CI includes 0; /hub/demand_unserved: '
     '1.2147 vs 1.5064; T04 6/6; results/S2_report.md'),
    ('F04', 'Review: perception', 'Crossing confusion matrix, false-clear episodes, entry delay, '
     'valid coverage; frames with lost image alignment become invalid and not clear',
     'Husan', 'Tuymurod', '2026-11-12', '2026-11-13', DONE, PENDING,
     'YOLO: 0 false clears in 13 occupied labels; HOG: 13 of 13 missed (equals always-clear '
     'baseline); evidence/m1a/real_video'),
    ('F05', 'Review: integration', 'S8 waits include passengers who never boarded (censored / left); '
     'Kaplan-Meier median; per-seed paired bootstrap; physical co-occupancy of crossing C',
     'Arkar', 'Aidil', '2026-11-13', '2026-11-13', DONE, PENDING,
     'smart_hub/s8_stats.py, s8_check, s8_summary; 20-episode run in evidence/s8'),
    ('F06', 'Review: routing', 'Live cost-only replan trial (traffic alone, road never blocked); '
     'S7 runs with 1 cm odometry noise instead of noise-free copies',
     'Aidil', 'Arkar', '2026-11-13', '2026-11-13', DONE, PENDING,
     'CHEAPER 52 s < 220 s (gain 76 %, needs 15 %); evidence/m4/s7'),
    ('F07', 'Review: bus', 'Test that reversing back into crossing C needs a new grant; docking '
     're-run with the stopping-distance rule and noisy observations',
     'Fazliddin', 'Arkar', '2026-11-13', '2026-11-13', DONE, PENDING,
     'test_safety_core.py; evidence/m2a/s3'),
    ('F08', 'Review: code quality', 'Pay the style debt in the M2-B and M3 files; no exemptions, '
     'no expected failures', 'Carla', 'Saidumar', '2026-11-13', '2026-11-13', DONE, PENDING,
     'flake8 and pep257 check every file; 0 xfail'),
    ('F09', 'Review: code quality', 'Stub smoke tests run in a ROS-free child interpreter instead '
     'of being skipped', 'Carla', 'Saidumar', '2026-11-13', '2026-11-13', DONE, PENDING,
     '9 tests run; they had been broken since the clean-shutdown change (fixed); 0 skipped'),
    ('F10', 'Review: safety', 'Reset handshake answers inside the 1 s client timeout '
     '(assignment 7.1); response time measured', 'Saidumar', 'Aidil', '2026-11-12',
     '2026-11-12', DONE, YES, '25 resets: mean 30.5 ms, max 357 ms; evidence/m5/s9_repeat'),
    ('F11', 'Review: safety', 'Numbered safety decision records replace the logger grace window; '
     'ambiguous cross-topic samples are kept, not dropped', 'Saidumar', 'Fazliddin',
     '2026-11-12', '2026-11-12', DONE, YES,
     '/hub/safety_decision; S9 repeat: 1799 records, 0 lost, 0 latched with a nonzero command'),
    ('F12', 'Review: safety', 'Speed-dependent stopping distance (v*tau + v^2/2a + margin) replaces '
     'the fixed 0.5 s look-ahead; other bodies predicted over the same time',
     'Saidumar', 'Fazliddin', '2026-11-12', '2026-11-12', DONE, YES,
     'safety_core.py; S3, S7, S9 and the 25-fault repetition pass after the change'),
    ('F13', 'Review: evidence', 'A crashed checker can no longer report PASSED '
     '(found while adding F06)', 'Saidumar', 'Aidil', '2026-11-13', '2026-11-13', DONE, YES,
     's3/s4/s6/s7/t04/t18 checkers'),
    ('F14', 'Release', 'License notices, third-party notices, exact dependency versions, '
     'SHA-256 manifest of evidence, models and datasets', 'Saidumar', 'Carla', '2026-11-12',
     '2026-11-12', DONE, YES, 'LICENSE, THIRD_PARTY_NOTICES.md, evidence/release'),
    ('F15', 'Release', 'Re-run every scenario and test on one commit and file the results '
     '(same-revision reproduction)', 'Saidumar', 'all', '2026-11-13', '2026-11-14', DOING, NA,
     'tools/run_release.sh; evidence/ stamped with one commit'),
    ('F16', 'Documentation', 'Function-level explanations in the nodes and core logic '
     '(about 150 functions)', 'Saidumar', 'all', '2026-11-13', '2026-11-13', DONE, YES,
     'docstrings; 394 tests, 0 failures, 0 skipped'),
    ('F17', 'Documentation', 'Each member reads the changes made in their subsystem and confirms '
     'or corrects them', 'all', 'Saidumar', '2026-11-14', '2026-11-16', TODO, PENDING,
     'README of each role; "Owner confirmed" column of this sheet'),
    ('F18', 'Report', 'Final report: section "Changes since the midterm" answering every review '
     'point, updated results and limitations, contribution table',
     'Saidumar', 'all', '2026-11-14', '2026-11-20', DOING, NA,
     'VIP_TeamShenanigans_PROJECT_Finalterm.pdf'),
    ('F19', 'Presentation', 'Presentation with a live Control Center demo; note the oral feedback',
     'all', 'Saidumar', '2026-11-17', '2026-11-17', TODO, NA, '2026-11-17 18:00'),
    ('F20', 'Presentation', 'Apply the oral feedback from the presentation', 'all', 'Saidumar',
     '2026-11-18', '2026-11-19', TODO, NA, 'new rows are added here after the presentation'),
    ('F21', 'Video', 'Record the final 20-30 minute video: every member explains their part and '
     'the changes; the UI is shown running tasks', 'all', 'Saidumar', '2026-11-18',
     '2026-11-19', TODO, NA, 'docs/demo/VIDEO_SCRIPT.md; tools/assemble_video.py'),
    ('F22', 'Submission', 'Build the upload set, check the rules (10 files, 1 GB, no links), '
     'upload; every member logs in and verifies', 'Saidumar', 'all', '2026-11-20',
     '2026-11-21', TODO, NA, 'tools/make_submission.sh; deadline 2026-11-21 18:00 KST'),
]

# review point, what was done, task ids, status
REVIEW = [
    ('Forecast target differs from the assignment (future queue vs no-boarding demand)',
     'Both forecasts published under their own names; T03 judged on the future queue; the '
     'small, uncertain gain there is reported as it is', 'F03', DONE),
    ('97 % crossing accuracy can hide false clears; class balance; false-clear timing',
     'Confusion matrix, always-clear baseline, false-clear episodes, entry delay', 'F04', DONE),
    ('A failed image alignment must not reuse a stale zone overlay',
     'After 5 lost frames the frame is invalid and the crossing is reported not clear', 'F04',
     DONE),
    ('Stabilisation worsens the labelled result (0.149 vs 0.114)',
     'All three conditions kept and reported; stabilisation stays the pre-registered main '
     'condition, not claimed as an improvement', 'F04', DONE),
    ('Boarded-only waits; add right-censored passengers and abandonment',
     'Still-waiting and left-queue passengers reported; Kaplan-Meier median', 'F05', DONE),
    ('Paired episode effect with uncertainty', 'Per-seed differences with a whole-episode '
     'bootstrap interval', 'F05', DONE),
    ('S8 checker does not record every pedestrian/agent co-occupancy of C',
     'Every co-occupancy episode recorded; a hub vehicle moving more than 0.5 s after a person '
     'entered is a violation', 'F05', DONE),
    ('No live cost-only replanning evidence', 'Live trial in the S7 check', 'F06', DONE),
    ('Nominal escort runs near-identical and noise-free', 'Seeded 1 cm odometry noise', 'F06',
     DONE),
    ('Reversal near C needs a new grant before re-entry', 'Supervisor-level test', 'F07', DONE),
    ('Remaining style debt; skipped and expected-failure tests are not passes',
     'All files pass both gates; 0 skipped, 0 xfail', 'F08, F09', DONE),
    ('Two-second reset handshake vs one-second client contract',
     'Server answers within 0.8 s; measured', 'F10', DONE),
    ('Logger ordering grace can hide a short violation',
     'Numbered decision records, no grace; ambiguous samples preserved', 'F11', DONE),
    ('Fixed 0.5 s look-ahead is not a stopping-distance guarantee',
     'Stopping-distance rule of assignment 3.3 with documented bounds', 'F12', DONE),
    ('Evidence spans different revisions; older 187-test summary conflicts',
     'One-commit release run; index points at the transcript', 'F14, F15', DOING),
    ('Raw lab video and detector weights absent; inference cannot be regenerated',
     'Both included in the ZIP with hashes and the exact command', 'F14', DONE),
    ('One scene, six occluded labels, unsurveyed floor scale; simulator has no inertia',
     'Not fixable before the deadline: stated as limitations in the report', 'F18', DOING),
    ('Individual mastery requires defence', 'Each member confirms their part and explains one '
     'stream, one action/transaction and one repaired failure', 'F17, F19', TODO),
]

TIMELINE = [
    ('2026-10-25', 'Midterm submission', DONE),
    ('2026-11-02', 'Midterm review received (draft 13.5 / 15)', DONE),
    ('2026-11-10', 'Individual defence', DONE),
    ('2026-11-11', 'Final assignment announced; plan made', DONE),
    ('2026-11-12', 'Control Center UI merged; M5 and M1-B review fixes', DONE),
    ('2026-11-13', 'M1-A, M2-A, M2-B, M3, M4 review fixes; code freeze; release run started', DOING),
    ('2026-11-16', 'Members confirm their parts; UI rehearsal', TODO),
    ('2026-11-17', 'Presentation, 18:00', TODO),
    ('2026-11-19', 'Oral feedback applied; video recorded', TODO),
    ('2026-11-20', 'Report, Team Plan and upload set final', TODO),
    ('2026-11-21', 'Upload before 18:00 KST; every member verifies', TODO),
]

DELIVERABLES = [
    ('VIP_TeamShenanigans_PROJECT_Finalterm.pdf', 'Main report', DOING),
    ('VIP_TeamShenanigans_PROJECT_Finalterm_Source.zip',
     'Source code, datasets (lab video, labels), models, evidence', DOING),
    ('VIP_TeamShenanigans_PROJECT_Finalterm_Video.mp4', '20-30 minute video, all members', TODO),
    ('VIP_TeamShenanigans_PROJECT_Finalterm_TeamPlan.xlsx', 'This workbook', DOING),
    ('VIP_TeamShenanigans_PROJECT_Finalterm_DemoClips.zip', 'Scenario and UI demo clips', TODO),
    ('SME_TeamShenanigans_PROJECT_Midterm.pdf', 'Midterm report, kept as submitted', DONE),
    ('SME_TeamShenanigans_PROJECT_Midterm_Video.mp4', 'Midterm video, kept as submitted', DONE),
]

FONT = 'Arial'
HEAD_FILL = PatternFill('solid', fgColor='1F3A5F')
FILLS = {DONE: 'D9F2E3', DOING: 'FFF2CC', TODO: 'F2F2F2', YES: 'D9F2E3', PENDING: 'FFF2CC'}
THIN = Side(style='thin', color='BFC7CF')
BORDER = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)


def table(ws, row, headers, rows, widths, status_cols=()):
    """Write a header and rows starting at `row`; return the first and last data row."""
    for c, (h, w) in enumerate(zip(headers, widths), 1):
        cell = ws.cell(row=row, column=c, value=h)
        cell.font = Font(name=FONT, bold=True, color='FFFFFF')
        cell.fill, cell.border = HEAD_FILL, BORDER
        cell.alignment = Alignment(vertical='center', wrap_text=True)
        ws.column_dimensions[get_column_letter(c)].width = w
    for r, values in enumerate(rows, row + 1):
        for c, value in enumerate(values, 1):
            cell = ws.cell(row=r, column=c, value=value)
            cell.font = Font(name=FONT, size=10)
            cell.border = BORDER
            cell.alignment = Alignment(vertical='top', wrap_text=True)
            if c in status_cols and value in FILLS:
                cell.fill = PatternFill('solid', fgColor=FILLS[value])
    ws.freeze_panes = ws.cell(row=row + 1, column=1)
    return row + 1, row + len(rows)


def title(ws, text, note):
    ws['A1'] = text
    ws['A1'].font = Font(name=FONT, size=14, bold=True)
    ws['A2'] = note
    ws['A2'].font = Font(name=FONT, size=10, italic=True, color='555555')


def main():
    wb = Workbook()
    summary = wb.active
    summary.title = 'Summary'
    tasks = wb.create_sheet('Tasks')
    title(tasks, 'Smart Mobility Hub - final-term task list (Team Shenanigans)',
          'Owner = the member responsible for that subsystem. The review fixes F03-F09 were '
          'implemented by Saidumar (M5) with Claude Code, with the owners\' permission; '
          '"Owner confirmed" turns to Yes when the owner has read and accepted the change. '
          'Status: Done / In progress / To do. Dates are YYYY-MM-DD.')
    first, last = table(
        tasks, 4, ['ID', 'Workstream', 'Task', 'Owner', 'Reviewer', 'Start', 'Due', 'Status',
                   'Owner confirmed', 'Result / evidence'],
        TASKS, [6, 20, 62, 12, 12, 12, 12, 13, 12, 60], status_cols=(8, 9))
    for col, options in (('H', [DONE, DOING, TODO]), ('I', [YES, PENDING, NA])):
        dv = DataValidation(type='list', formula1='"%s"' % ','.join(options), allow_blank=False)
        tasks.add_data_validation(dv)
        dv.add('%s%d:%s%d' % (col, first, col, last))
    rng = {c: "Tasks!$%s$%d:$%s$%d" % (c, first, c, last) for c in 'DHI'}

    title(summary, 'Smart Mobility Hub - finalized Team Plan',
          'Team Shenanigans, team leader Egamberdiev Saidumar. Counts below are formulas over '
          'the Tasks sheet: change a status there and they update.')
    rows = [('Tasks in total', '=COUNTA(Tasks!$A$%d:$A$%d)' % (first, last)),
            ('Done', '=COUNTIF(%s,"%s")' % (rng['H'], DONE)),
            ('In progress', '=COUNTIF(%s,"%s")' % (rng['H'], DOING)),
            ('To do', '=COUNTIF(%s,"%s")' % (rng['H'], TODO)),
            ('Share done', '=IF(B5=0,0,B6/B5)'),
            ('Done but waiting for the owner to confirm',
             '=COUNTIFS(%s,"%s",%s,"%s")' % (rng['H'], DONE, rng['I'], PENDING))]
    for r, (label, formula) in enumerate(rows, 5):
        summary.cell(row=r, column=1, value=label).font = Font(name=FONT, size=11)
        cell = summary.cell(row=r, column=2, value=formula)
        cell.font = Font(name=FONT, size=11, bold=True)
    summary['B9'].number_format = '0%'
    m_first, m_last = table(
        summary, 13, ['Member', 'Role', 'Subsystem', 'Tasks owned', 'Done', 'To confirm'],
        [(n, role, what,
          '=COUNTIF(%s,"%s")' % (rng['D'], n),
          '=COUNTIFS(%s,"%s",%s,"%s")' % (rng['D'], n, rng['H'], DONE),
          '=COUNTIFS(%s,"%s",%s,"%s")' % (rng['D'], n, rng['I'], PENDING))
         for n, role, what in MEMBERS], [44, 10, 62, 13, 10, 13])
    note = summary.cell(row=m_last + 2, column=1,
                        value='Tasks owned by "all" are counted for nobody in this table; '
                              'they are F02, F17, F19, F20 and F21 on the Tasks sheet.')
    note.font = Font(name=FONT, size=10, italic=True, color='555555')
    summary.freeze_panes = None

    review = wb.create_sheet('Review response')
    title(review, 'Midterm review: what was asked and what was done',
          'Source: review of 2026-11-02 (SMSL_Team_E_Review.pdf). Task IDs refer to the '
          'Tasks sheet.')
    table(review, 4, ['Review point', 'What was done', 'Tasks', 'Status'], REVIEW,
          [62, 70, 12, 13], status_cols=(4,))

    timeline = wb.create_sheet('Timeline')
    title(timeline, 'Milestones', 'Dates are YYYY-MM-DD; times are KST.')
    table(timeline, 4, ['Date', 'Milestone', 'Status'], TIMELINE, [14, 80, 13], status_cols=(3,))

    files = wb.create_sheet('Deliverables')
    title(files, 'Upload set (at most 10 files, each at most 1 GB, no external links)',
          'Built and checked by tools/make_submission.sh. The team leader uploads; every '
          'member then logs in and verifies.')
    table(files, 4, ['File', 'Content', 'Status'], DELIVERABLES, [56, 62, 13], status_cols=(3,))

    wb.calculation.fullCalcOnLoad = True      # formulas are computed when the file opens
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    wb.save(OUT)
    done = sum(t[7] == DONE for t in TASKS)
    print('wrote %s: %d tasks (%d done, %d in progress, %d to do), %d review points'
          % (os.path.relpath(OUT), len(TASKS), done, sum(t[7] == DOING for t in TASKS),
             sum(t[7] == TODO for t in TASKS), len(REVIEW)))


if __name__ == '__main__':
    main()
