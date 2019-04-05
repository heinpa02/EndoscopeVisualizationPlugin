/*===================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/

#include <QStringList>

#include "QmitkImageStatisticsTreeItem.h"

QmitkImageStatisticsTreeItem::QmitkImageStatisticsTreeItem(
  mitk::ImageStatisticsContainer::ImageStatisticsObject statisticsData,
  mitk::ImageStatisticsContainer::ImageStatisticsObject::StatisticNameVector statisticNames,
  QVariant label,
  QmitkImageStatisticsTreeItem *parent)
{
  m_parentItem = parent;
  m_statistics = statisticsData;
  m_statisticNames = statisticNames;
  m_label = label;
}

QmitkImageStatisticsTreeItem::~QmitkImageStatisticsTreeItem()
{
  qDeleteAll(m_childItems);
}

void QmitkImageStatisticsTreeItem::appendChild(QmitkImageStatisticsTreeItem *item)
{
  m_childItems.append(item);
}

QmitkImageStatisticsTreeItem *QmitkImageStatisticsTreeItem::child(int row)
{
  return m_childItems.value(row);
}

int QmitkImageStatisticsTreeItem::childCount() const
{
  return m_childItems.count();
}

int QmitkImageStatisticsTreeItem::columnCount() const
{
  return m_statisticNames.size() + 1;
}

QVariant QmitkImageStatisticsTreeItem::data(int column) const
{
  QVariant result;
  if (column > 0 && !m_statisticNames.empty())
  {
    if (column - 1 < m_statisticNames.size())
    {
      auto statisticKey = m_statisticNames.at(column - 1);
      std::stringstream ss;
      if (m_statistics.HasStatistic(statisticKey))
      {
        ss << m_statistics.GetValueNonConverted(statisticKey);
      }
      else
      {
        return QVariant();
      }
      result = QVariant(QString::fromStdString(ss.str()));
    }
    else
    {
      return QVariant();
    }
  }
  else if (column == 0)
  {
    result = m_label;
  }
  return result;
}

QmitkImageStatisticsTreeItem *QmitkImageStatisticsTreeItem::parentItem()
{
  return m_parentItem;
}

int QmitkImageStatisticsTreeItem::row() const
{
  if (m_parentItem)
    return m_parentItem->m_childItems.indexOf(const_cast<QmitkImageStatisticsTreeItem *>(this));

  return 0;
}
