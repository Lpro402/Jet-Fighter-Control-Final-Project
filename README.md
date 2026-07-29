# Jet Fighter Control — Final Project

פרויקט גמר בבקרת התנועה הרוחבית של מטוס קרב. המאגר מרכז את קובצי MATLAB העדכניים של ההגשה, הדוח הסופי, גרפים ותוצאות, וכן את קוד המקור של הסימולטור האינטראקטיבי.

## סימולטור מקוון

הסימולטור זמין ללא תשלום בקישור הקבוע:

https://advanced-aircraft-simulator.l2shonak.chatgpt.site

## מבנה המאגר

- `matlab/` — קובצי `newshela*.m` העדכניים, לפי סעיפי הפרויקט.
- `report/` — הדוח הסופי בפורמטים Word ו־PDF.
- `figures/` — הגרפים ששולבו בדוח.
- `results/` — תוצאות MATLAB בפורמטים MAT ו־CSV.
- `requirements/` — מסמך דרישות הפרויקט.
- `simulator/` — קוד המקור המלא של הסימולטור המקוון.

## קובצי MATLAB

| קובץ | סעיף |
|---|---|
| `newshela1.m` | שאלה 1 |
| `newshela2_1.m` | שאלה 2(a) |
| `newshela2_2.m` | שאלה 2(b) |
| `newshela2_3.m` | שאלה 2(c) |
| `newshela3_1.m` | שאלה 3(a) |
| `newshela3_2.m` | שאלה 3(b) |
| `newshela3_3.m` | שאלה 3(c) |
| `newshela3_4.m` | שאלה 3(d) |
| `newshela4.m` | שאלה 4 |

הקבצים מיועדים ל־MATLAB R2025b עם Control System Toolbox.

## הרצת הסימולטור מקומית

```powershell
cd simulator
npm install
npm run dev
```

לאחר מכן יש לפתוח בדפדפן את הכתובת המקומית שמודפסת במסוף.

## הערת מקור

קובצי MATLAB הרשמיים במאגר הם קובצי `newshela*.m`. קובצי תכן חלופיים שנוצרו במהלך העבודה אינם נכללים בתיקיית MATLAB, כדי לשמור על מקור הגשה ברור ואחיד.
