# Board1 full referee QR validation

Date: 2026-10-16. ROS Melodic / Python 2.7. No new runtime installed.

Validated runtime commit: a49fe6d6c47d6fa694cda9bb95e96da48eee0329

17 original QR assets plus empty frame, 68 four-slot decoding cases, 14641 selection combinations: PASS.
8 real camera scenes x 3 rounds: PASS. All-invalid scenes produce no task during each 5-second observation.

| Scene | Result | Expected message |
| --- | --- | --- |
| all_invalid | PASS (3 rounds) | None |
| invalid_acd | PASS (3 rounds) | [1, 1, 1, 3, 1, 1] |
| invalid_d | PASS (3 rounds) | [1, 1, 1, 3, 0, 2] |
| invalid_url | PASS (3 rounds) | [1, 1, 1, 3, 2, 4] |
| labelled_1 | PASS (3 rounds) | [0, 1, 1, 2, 3, 0] |
| labelled_2 | PASS (3 rounds) | [1, 1, 1, 3, 2, 0] |
| plain_1 | PASS (3 rounds) | [0, 1, 1, 2, 3, 0] |
| plain_2 | PASS (3 rounds) | [1, 1, 1, 3, 2, 0] |

Original assets, replay tools and evidence: tests/full_referee_qr/
Backup: /home/EPRobot/code_archives/board1_full_qr/20261016_152845

The runtime recognition script remains the verified implementation. These QR assets are regression fixtures; runtime decoding does not require template matching.
Raw payload ACD, D and https://www.hlcode.cn are invalid tasks. The third distractor is visually labelled BD but actually encodes a URL.
No navigation was started. Vision returned to state 9. Fixed-view lighting/distance only; moving-car recognition is not validated.
