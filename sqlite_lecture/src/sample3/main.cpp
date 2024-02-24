#include <sqlite3.h>
#include <sqlite_lecture/relation_data.hpp>

int exec_callback(void * data, int argc, char ** argv, char ** azColName)
{
  RelationData * relation_data = (RelationData *)data;
  relation_data->increment();
  for (int i = 0; i < argc; i++) {
    relation_data->setAtBack(azColName[i], argv[i]);
  }
  return 0;
}

int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  printf("Start\n");

  sqlite3 * db = NULL;
  if (sqlite3_open("data.db", &db) != SQLITE_OK) {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // create Table
  if (sqlite3_exec(
      db, "CREATE TABLE IF NOT EXISTS PositionList (name PRIMARY KEY, x, y);", NULL,
      NULL, NULL) != SQLITE_OK)
  {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // insert record with statement
  char sql_raw[256] = "INSERT OR IGNORE INTO PositionList VALUES (?, ?, ?);";
  sqlite3_stmt * pStmt = NULL;
  if (sqlite3_prepare_v2(db, sql_raw, 256, &pStmt, NULL) != SQLITE_OK) {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }

  // 1st data
  sqlite3_bind_text(pStmt, 1, "pos1", -1, SQLITE_TRANSIENT);
  sqlite3_bind_double(pStmt, 2, 1.1);
  sqlite3_bind_double(pStmt, 3, 2.2);
  while (SQLITE_DONE != sqlite3_step(pStmt)) {
  }
  sqlite3_reset(pStmt);

  // 2nd data
  sqlite3_bind_text(pStmt, 1, "pos2", -1, SQLITE_TRANSIENT);
  sqlite3_bind_double(pStmt, 2, 5.1);
  sqlite3_bind_double(pStmt, 3, 6.2);
  while (SQLITE_DONE != sqlite3_step(pStmt)) {
  }
  sqlite3_finalize(pStmt);

  // get & view record
  RelationData relation_data;
  if (sqlite3_exec(db, "SELECT * FROM PositionList", exec_callback, (void *)&relation_data, NULL)) {
    printf("%s[%s]\n", __func__, sqlite3_errmsg(db));
    return -1;
  }
  relation_data.print();

  printf("Done\n");
  return 0;
}
